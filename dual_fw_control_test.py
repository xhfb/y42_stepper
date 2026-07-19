"""Y42 Emm / X 双固件控制实测 (COM9).

流程:
  1. 记录当前固件
  2. 切到 Emm → enable / jog / 脉冲位置 / stop / disable
  3. 切到 X   → enable / jog / 直通位置 / 梯形位置 / 力矩限速 / stop / disable
  4. 恢复原固件
"""

from __future__ import annotations

import sys
import time
from typing import List, Tuple

from serial import Serial

from y42_stepper import Direction, FirmwareType, MotionMode, Y42Device


def section(title: str) -> None:
    print(f"\n{'=' * 60}\n{title}\n{'=' * 60}")


def check(ok: bool, name: str, detail: str = "") -> bool:
    mark = "PASS" if ok else "FAIL"
    extra = f" | {detail}" if detail else ""
    print(f"  [{mark}] {name}{extra}")
    return ok


def test_emm(motor: Y42Device) -> List[Tuple[str, bool]]:
    results: List[Tuple[str, bool]] = []
    section(f"Emm 固件控制  (detected={motor.firmware_type.name})")

    ok = motor.firmware_type == FirmwareType.EMM_FIRMWARE
    results.append(("firmware_is_emm", check(ok, "firmware_type", motor.firmware_type.name)))
    if not ok:
        return results

    results.append(("enable", check(motor.enable(), "enable")))

    v0 = motor.get_bus_voltage()
    pos0 = motor.get_realtime_position()
    results.append(("read_status", check(True, "bus/pos", f"{v0} mV, pos={pos0:.2f}°")))

    ok = motor.jog(speed_rpm=80, direction=Direction.CW, acceleration=15)
    time.sleep(0.5)
    spd = motor.get_realtime_speed()
    motor.stop()
    results.append(("jog_cw", check(ok and abs(spd) > 5, "jog 80rpm", f"speed={spd}")))

    ok = motor.move_position(
        pulses=1600,
        speed_rpm=200,
        acceleration=20,
        motion_mode=MotionMode.RELATIVE_CURRENT,
    )
    reached = motor.wait_position_reached(timeout=8)
    pos1 = motor.get_realtime_position()
    results.append(
        (
            "move_pulses_1600",
            check(ok and reached, "move 1600 pulses (~180°)", f"reached={reached}, pos={pos1:.2f}°"),
        )
    )

    ok = motor.move_position(
        angle_deg=90,
        speed_rpm=150,
        acceleration=15,
        motion_mode=MotionMode.RELATIVE_CURRENT,
    )
    reached = motor.wait_position_reached(timeout=8)
    pos2 = motor.get_realtime_position()
    results.append(
        (
            "move_angle_90",
            check(ok and reached, "move 90° via pulses", f"reached={reached}, pos={pos2:.2f}°"),
        )
    )

    results.append(("disable", check(motor.disable(), "disable")))
    return results


def test_x(motor: Y42Device) -> List[Tuple[str, bool]]:
    results: List[Tuple[str, bool]] = []
    section(f"X 固件控制  (detected={motor.firmware_type.name})")

    ok = motor.firmware_type == FirmwareType.X_FIRMWARE
    results.append(("firmware_is_x", check(ok, "firmware_type", motor.firmware_type.name)))
    if not ok:
        return results

    results.append(("supports_torque", check(motor.supports_torque, "supports_torque")))
    results.append(("enable", check(motor.enable(), "enable")))

    cfg = motor.get_config()
    results.append(("get_config", check(cfg is not None, "get_config", type(cfg).__name__)))

    ok = motor.jog(speed_rpm=80, direction=Direction.CCW, acceleration=800)
    time.sleep(0.5)
    spd = motor.get_realtime_speed()
    motor.stop()
    results.append(("jog_ccw", check(ok and abs(spd) > 5, "jog 80rpm", f"speed={spd}")))

    ok = motor.jog(
        speed_rpm=60,
        direction=Direction.CW,
        acceleration=600,
        max_current_ma=1500,
    )
    time.sleep(0.4)
    motor.stop()
    results.append(("jog_current_limit", check(ok, "jog + max_current 1500mA")))

    ok = motor.move_position(
        angle_deg=45,
        speed_rpm=200,
        acceleration=800,
        mode="direct",
        motion_mode=MotionMode.RELATIVE_CURRENT,
    )
    reached = motor.wait_position_reached(timeout=8)
    pos = motor.get_realtime_position()
    results.append(
        (
            "move_direct_45",
            check(ok and reached, "direct position 45°", f"reached={reached}, pos={pos:.2f}°"),
        )
    )

    ok = motor.move_position(
        angle_deg=-60,
        speed_rpm=180,
        acceleration=500,
        decel=500,
        mode="trap",
        motion_mode=MotionMode.RELATIVE_CURRENT,
    )
    reached = motor.wait_position_reached(timeout=8)
    pos = motor.get_realtime_position()
    results.append(
        (
            "move_trap_-60",
            check(ok and reached, "trap position -60°", f"reached={reached}, pos={pos:.2f}°"),
        )
    )

    ok = motor.torque_limited(
        current_ma=400,
        max_speed_rpm=80,
        direction=Direction.CW,
        slope_ma_s=200,
    )
    time.sleep(0.4)
    motor.stop()
    results.append(("torque_limited", check(ok, "torque 400mA / max 80rpm")))

    results.append(("disable", check(motor.disable(), "disable")))
    return results


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
    print(f"串口 {port} @ 115200 — Y42 双固件控制测试")

    ser = Serial(port, 115200, timeout=0.15)
    all_results: List[Tuple[str, bool]] = []
    original: FirmwareType | None = None

    try:
        motor = Y42Device(ser, address=1)
        original = motor.detect_firmware()
        ver = motor.get_version()
        section(f"初始状态  fw={original.name}  ver={ver.firmware_version_str}  hw={ver.hw_type_str}")

        # --- Emm ---
        if original != FirmwareType.EMM_FIRMWARE:
            ok = motor.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
            check(ok, "switch_to_emm", motor.firmware_type.name)
            if not ok:
                print("无法切到 Emm，中止")
                return 2
        all_results.extend(test_emm(motor))

        # --- X ---
        ok = motor.switch_firmware(FirmwareType.X_FIRMWARE, store=True)
        check(ok, "switch_to_x", motor.firmware_type.name)
        if not ok:
            print("无法切到 X，跳过 X 测试")
        else:
            all_results.extend(test_x(motor))

    finally:
        try:
            if original is not None:
                section(f"恢复原固件 {original.name}")
                m = Y42Device(ser, address=1, auto_test=False)
                m.detect_firmware()
                m.stop()
                ok = m.switch_firmware(original, store=True)
                check(ok, "restore_firmware", m.firmware_type.name)
                m.disable()
        except Exception as e:
            print(f"  [WARN] 恢复固件异常: {e}")
        ser.close()

    section("汇总")
    passed = sum(1 for _, ok in all_results if ok)
    failed = sum(1 for _, ok in all_results if not ok)
    print(f"total={len(all_results)}  pass={passed}  fail={failed}")
    for name, ok in all_results:
        if not ok:
            print(f"  FAIL: {name}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
