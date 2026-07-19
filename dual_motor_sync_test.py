"""Y42 双电机多机控制实测 (地址1+2, X/Emm).

测试项:
  - 广播同步触发 (sync 缓存 + FF 66)
  - 多电机命令 0xAA 一包双轴
  - 分别在 Emm / X 固件下各跑一轮
"""

from __future__ import annotations

import sys
import time
from typing import List, Tuple

from serial import Serial

from y42_stepper import Direction, FirmwareType, MotionMode, Y42Device
from y42_stepper.commands import EmmJog, EmmPosition, XJog, XPositionDirect, build_command_frame
from y42_stepper.configs import Code, Protocol, SyncFlag, add_checksum
from y42_stepper.parameters import (
    DeviceParams,
    EmmJogParams,
    EmmPositionParams,
    XJogParams,
    XPositionDirectParams,
)


def check(ok: bool, name: str, detail: str = "") -> bool:
    mark = "PASS" if ok else "FAIL"
    extra = f" | {detail}" if detail else ""
    print(f"  [{mark}] {name}{extra}")
    return ok


def section(title: str) -> None:
    print(f"\n{'=' * 60}\n{title}\n{'=' * 60}")


def switch_both(
    m1: Y42Device, m2: Y42Device, fw: FirmwareType
) -> bool:
    m1.stop()
    m2.stop()
    time.sleep(0.05)
    ok1 = m1.switch_firmware(fw, store=True)
    ok2 = m2.switch_firmware(fw, store=True)
    return check(
        ok1 and ok2,
        f"switch_both_to_{fw.name}",
        f"m1={m1.firmware_type.name} m2={m2.firmware_type.name}",
    )


def test_sync_trigger(m1: Y42Device, m2: Y42Device, fw: FirmwareType) -> List[Tuple[str, bool]]:
    """sync 缓存 + 广播 FF66 齐发."""
    out: List[Tuple[str, bool]] = []
    tag = fw.name

    m1.enable()
    m2.enable()
    time.sleep(0.05)

    if fw == FirmwareType.EMM_FIRMWARE:
        ok1 = m1.jog(speed_rpm=60, direction=Direction.CW, acceleration=12, sync=True)
        ok2 = m2.jog(speed_rpm=60, direction=Direction.CCW, acceleration=12, sync=True)
    else:
        ok1 = m1.jog(speed_rpm=60, direction=Direction.CW, acceleration=800, sync=True)
        ok2 = m2.jog(speed_rpm=60, direction=Direction.CCW, acceleration=800, sync=True)

    out.append((f"{tag}_sync_cache_jog", check(ok1 and ok2, f"[{tag}] cache jog both")))

    # 触发前转速应接近 0
    time.sleep(0.1)
    s1a = abs(m1.get_realtime_speed())
    s2a = abs(m2.get_realtime_speed())
    out.append(
        (
            f"{tag}_pre_trigger_still",
            check(s1a < 15 and s2a < 15, f"[{tag}] before trigger idle", f"s1={s1a} s2={s2a}"),
        )
    )

    ok = Y42Device.sync_move(m1.device_params)
    time.sleep(0.45)
    s1b = abs(m1.get_realtime_speed())
    s2b = abs(m2.get_realtime_speed())
    out.append(
        (
            f"{tag}_sync_trigger_move",
            check(
                ok and s1b > 10 and s2b > 10,
                f"[{tag}] FF66 both moving",
                f"ok={ok} s1={s1b} s2={s2b}",
            ),
        )
    )

    m1.stop()
    m2.stop()
    time.sleep(0.1)

    # 位置同步: 相对运动不同脉冲/角度
    if fw == FirmwareType.EMM_FIRMWARE:
        ok1 = m1.move_position(pulses=800, speed_rpm=180, acceleration=20, sync=True)
        ok2 = m2.move_position(pulses=800, speed_rpm=180, acceleration=20, sync=True)
    else:
        ok1 = m1.move_position(
            angle_deg=45, speed_rpm=180, acceleration=600, mode="direct", sync=True
        )
        ok2 = m2.move_position(
            angle_deg=45, speed_rpm=180, acceleration=600, mode="direct", sync=True
        )
    out.append((f"{tag}_sync_cache_pos", check(ok1 and ok2, f"[{tag}] cache position")))

    Y42Device.sync_move(m1.device_params)
    r1 = m1.wait_position_reached(timeout=8)
    r2 = m2.wait_position_reached(timeout=8)
    out.append(
        (
            f"{tag}_sync_pos_reached",
            check(r1 and r2, f"[{tag}] both reached", f"r1={r1} r2={r2}"),
        )
    )

    m1.disable()
    m2.disable()
    return out


def test_multi_motor_aa(
    m1: Y42Device, m2: Y42Device, fw: FirmwareType
) -> List[Tuple[str, bool]]:
    """00 AA 一包发两轴命令."""
    out: List[Tuple[str, bool]] = []
    tag = fw.name
    cs = m1.device_params.checksum_mode

    m1.enable()
    m2.enable()
    time.sleep(0.05)

    # 使能帧也可打进 AA，这里直接发运动
    if fw == FirmwareType.EMM_FIRMWARE:
        body1 = bytes([1, Code.JOG]) + EmmJogParams(
            direction=Direction.CW, speed=50, acceleration=10, sync_flag=SyncFlag.IMMEDIATE
        ).bytes
        body2 = bytes([2, Code.JOG]) + EmmJogParams(
            direction=Direction.CCW, speed=50, acceleration=10, sync_flag=SyncFlag.IMMEDIATE
        ).bytes
    else:
        body1 = bytes([1, Code.JOG]) + XJogParams(
            direction=Direction.CW,
            acceleration_rpm_s=800,
            speed_raw=500,  # 50.0 rpm
            sync=SyncFlag.IMMEDIATE,
        ).bytes
        body2 = bytes([2, Code.JOG]) + XJogParams(
            direction=Direction.CCW,
            acceleration_rpm_s=800,
            speed_raw=500,
            sync=SyncFlag.IMMEDIATE,
        ).bytes

    f1 = build_command_frame(body1, cs)
    f2 = build_command_frame(body2, cs)
    ok = Y42Device.multi_motor([f1, f2], m1.device_params, expect_ack=True)
    time.sleep(0.4)
    s1 = abs(m1.get_realtime_speed())
    s2 = abs(m2.get_realtime_speed())
    out.append(
        (
            f"{tag}_aa_jog",
            check(ok and s1 > 5 and s2 > 5, f"[{tag}] AA jog both", f"s1={s1} s2={s2}"),
        )
    )

    # 停止: 两轴 FE
    stop1 = add_checksum(bytes([1, Code.ESTOP, Protocol.ESTOP, 0]), cs)
    stop2 = add_checksum(bytes([2, Code.ESTOP, Protocol.ESTOP, 0]), cs)
    Y42Device.multi_motor([stop1, stop2], m1.device_params, expect_ack=True)
    time.sleep(0.15)

    # AA 位置
    if fw == FirmwareType.EMM_FIRMWARE:
        b1 = bytes([1, Code.POSITION]) + EmmPositionParams(
            direction=Direction.CW,
            speed=150,
            acceleration=15,
            pulse_count=640,
            motion_mode=MotionMode.RELATIVE_CURRENT,
            sync_flag=SyncFlag.IMMEDIATE,
        ).bytes
        b2 = bytes([2, Code.POSITION]) + EmmPositionParams(
            direction=Direction.CCW,
            speed=150,
            acceleration=15,
            pulse_count=640,
            motion_mode=MotionMode.RELATIVE_CURRENT,
            sync_flag=SyncFlag.IMMEDIATE,
        ).bytes
    else:
        b1 = bytes([1, Code.POSITION_DIRECT]) + XPositionDirectParams(
            direction=Direction.CW,
            speed_raw=1500,
            angle_raw=300,  # 30.0°
            motion_mode=MotionMode.RELATIVE_CURRENT,
            sync=SyncFlag.IMMEDIATE,
        ).bytes
        b2 = bytes([2, Code.POSITION_DIRECT]) + XPositionDirectParams(
            direction=Direction.CCW,
            speed_raw=1500,
            angle_raw=300,
            motion_mode=MotionMode.RELATIVE_CURRENT,
            sync=SyncFlag.IMMEDIATE,
        ).bytes

    ok = Y42Device.multi_motor(
        [build_command_frame(b1, cs), build_command_frame(b2, cs)],
        m1.device_params,
        expect_ack=True,
    )
    r1 = m1.wait_position_reached(timeout=8)
    r2 = m2.wait_position_reached(timeout=8)
    out.append(
        (
            f"{tag}_aa_position",
            check(ok and r1 and r2, f"[{tag}] AA position both", f"r1={r1} r2={r2}"),
        )
    )

    m1.disable()
    m2.disable()
    return out


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
    print(f"串口 {port} — 双电机(1,2) 多机控制 / X+Emm")

    ser = Serial(port, 115200, timeout=0.15)
    results: List[Tuple[str, bool]] = []
    originals: dict[int, FirmwareType] = {}

    try:
        m1 = Y42Device(ser, address=1)
        m2 = Y42Device(ser, address=2)
        originals[1] = m1.detect_firmware()
        originals[2] = m2.detect_firmware()
        v1 = m1.get_version()
        v2 = m2.get_version()
        section(
            f"初始 m1={originals[1].name}/{v1.firmware_version_str}  "
            f"m2={originals[2].name}/{v2.firmware_version_str}"
        )

        for fw in (FirmwareType.EMM_FIRMWARE, FirmwareType.X_FIRMWARE):
            section(f"固件 {fw.name}")
            if not switch_both(m1, m2, fw):
                results.append((f"switch_{fw.name}", False))
                continue
            results.extend(test_sync_trigger(m1, m2, fw))
            results.extend(test_multi_motor_aa(m1, m2, fw))

    finally:
        try:
            section("恢复各轴原固件")
            for addr, fw in originals.items():
                m = Y42Device(ser, address=addr, auto_test=False)
                m.detect_firmware()
                m.stop()
                ok = m.switch_firmware(fw, store=True)
                check(ok, f"restore_addr{addr}", f"{fw.name} -> {m.firmware_type.name}")
                m.disable()
        except Exception as e:
            print(f"  [WARN] restore: {e}")
        ser.close()

    section("汇总")
    passed = sum(1 for _, ok in results if ok)
    failed = sum(1 for _, ok in results if not ok)
    print(f"total={len(results)} pass={passed} fail={failed}")
    for name, ok in results:
        if not ok:
            print(f"  FAIL: {name}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
