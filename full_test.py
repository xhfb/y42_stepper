"""Y42 双固件驱动库实机冒烟测试.

默认只做只读探测；运动 / 写参 / 固件切换需显式打开开关。

用法:
  python full_test.py COM3
  python full_test.py COM3 --move
  python full_test.py COM3 --firmware          # 切固件再切回（store=False）
  python full_test.py COM3 --full-safe
  python full_test.py COM3 --addrs 1,2 --dual
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback
from dataclasses import dataclass
from typing import Callable, List, Optional

from serial import Serial

from y42_stepper import (
    Direction,
    FirmwareType,
    HomingMode,
    MotionMode,
    Y42Device,
)
from y42_stepper.configs import Code


@dataclass
class CaseResult:
    name: str
    ok: bool
    detail: str
    skipped: bool = False


def _ok(name: str, detail: str = "") -> CaseResult:
    return CaseResult(name, True, detail, False)


def _skip(name: str, reason: str) -> CaseResult:
    return CaseResult(name, True, f"SKIP ({reason})", True)


def _fail(name: str, detail: str) -> CaseResult:
    return CaseResult(name, False, detail, False)


def _run(name: str, fn: Callable[[], str]) -> CaseResult:
    try:
        return _ok(name, fn())
    except Exception as e:
        return _fail(name, f"{type(e).__name__}: {e}")


def _require(ok: bool, msg: str) -> None:
    if not ok:
        raise AssertionError(msg)


@dataclass
class Flags:
    write: bool = False
    move: bool = False
    timed: bool = False
    home: bool = False
    firmware: bool = False
    calibrate: bool = False
    restart: bool = False
    dual: bool = False


def test_detect(motor: Y42Device) -> List[CaseResult]:
    out: List[CaseResult] = []

    def _ver() -> str:
        v = motor.get_version()
        _require(v is not None, "version is None")
        return f"{v.firmware_version_str} / {v.hw_type_str}"

    out.append(_run("get_version", _ver))

    def _fw() -> str:
        fw = motor.detect_firmware()
        return f"{fw.name} supports_torque={motor.supports_torque}"

    out.append(_run("detect_firmware", _fw))

    def _opt() -> str:
        o = motor.get_option_status()
        # 0x1A FwType 不可靠，仅展示原始位，以 detect_firmware 为准
        return (
            f"opt_fw_bit_emm={o.firmware_is_emm} closed={o.closed_loop} "
            f"scale={o.scale_input} lock={o.lock_param_level} "
            f"(fw_bit不可靠,实际={motor.firmware_type.name})"
        )

    out.append(_run("get_option_status", _opt))
    return out


def test_reads(motor: Y42Device) -> List[CaseResult]:
    out: List[CaseResult] = []
    reads = [
        ("get_bus_voltage", lambda: f"{motor.get_bus_voltage()} mV"),
        ("get_bus_current", lambda: f"{motor.get_bus_current()} mA"),
        ("get_phase_current", lambda: f"{motor.get_phase_current()} mA"),
        ("get_battery_voltage", lambda: f"{motor.get_battery_voltage()} mV"),
        ("get_temperature", lambda: f"{motor.get_temperature()} C"),
        ("get_encoder_degrees", lambda: f"{motor.get_encoder_degrees():.3f} deg"),
        ("get_realtime_position", lambda: f"{motor.get_realtime_position():.3f} deg"),
        ("get_realtime_speed", lambda: f"{motor.get_realtime_speed()} RPM"),
        ("get_position_error", lambda: f"{motor.get_position_error():.4f} deg"),
        ("get_motor_status", lambda: str(motor.get_motor_status())),
        ("get_home_motor_status", lambda: str(motor.get_home_motor_status())),
        ("get_io_status", lambda: str(motor.get_io_status())),
        ("get_position_window", lambda: f"{motor.get_position_window()} deg"),
        ("get_pid", lambda: str(motor.get_pid())),
        ("get_config", lambda: type(motor.get_config()).__name__),
        ("get_system_status", lambda: type(motor.get_system_status()).__name__),
    ]
    for name, fn in reads:
        out.append(_run(name, fn))
    return out


def test_move(motor: Y42Device, flags: Flags) -> List[CaseResult]:
    if not flags.move:
        return [_skip("move", "加 --move")]

    out: List[CaseResult] = []

    def _en() -> str:
        _require(motor.enable(), "enable failed")
        return "ok"

    out.append(_run("enable", _en))

    def _jog() -> str:
        _require(motor.jog(speed_rpm=60, direction=Direction.CW), "jog failed")
        time.sleep(0.4)
        _require(motor.stop(), "stop failed")
        return "jog+stop"

    out.append(_run("jog", _jog))

    def _pos() -> str:
        if motor.firmware_type == FirmwareType.EMM_FIRMWARE:
            ok = motor.move_position(pulses=800, speed_rpm=200, acceleration=20)
        else:
            ok = motor.move_position(
                angle_deg=45,
                speed_rpm=200,
                acceleration=800,
                mode="trap",
            )
        _require(ok, "move_position failed")
        reached = motor.wait_position_reached(timeout=8)
        return f"reached={reached}"

    out.append(_run("move_position", _pos))

    if motor.supports_torque:

        def _tq() -> str:
            _require(
                motor.torque_limited(
                    current_ma=400,
                    max_speed_rpm=100,
                    slope_ma_s=200,
                ),
                "torque failed",
            )
            time.sleep(0.3)
            _require(motor.stop(), "stop after torque failed")
            return "ok"

        out.append(_run("torque_limited", _tq))
    else:
        out.append(_skip("torque_limited", "当前为 Emm 固件"))

    def _dis() -> str:
        _require(motor.disable(), "disable failed")
        return "ok"

    out.append(_run("disable", _dis))
    return out


def test_firmware_switch(motor: Y42Device, flags: Flags) -> List[CaseResult]:
    if not flags.firmware:
        return [_skip("switch_firmware", "加 --firmware")]

    out: List[CaseResult] = []
    original = motor.detect_firmware()
    other = (
        FirmwareType.EMM_FIRMWARE
        if original == FirmwareType.X_FIRMWARE
        else FirmwareType.X_FIRMWARE
    )

    def _switch() -> str:
        # Y42: store=False 时 D5 常返回成功但不生效，必须 store=True
        motor.stop()
        time.sleep(0.05)
        _require(motor.switch_firmware(other, store=True), f"switch to {other} failed")
        _require(motor.firmware_type == other, "profile not updated")
        pos = motor.get_realtime_position()
        cfg = type(motor.get_config()).__name__
        _require(
            motor.switch_firmware(original, store=True),
            f"restore {original} failed",
        )
        return f"{original.name}->{other.name}->{original.name}; pos={pos:.2f}; cfg={cfg}"

    out.append(_run("switch_firmware_roundtrip", _switch))

    if flags.move:

        def _move_both() -> str:
            notes = []
            for fw in (FirmwareType.X_FIRMWARE, FirmwareType.EMM_FIRMWARE):
                motor.stop()
                _require(motor.switch_firmware(fw, store=True), f"to {fw} failed")
                motor.enable()
                if fw == FirmwareType.EMM_FIRMWARE:
                    ok = motor.move_position(pulses=400, speed_rpm=150)
                else:
                    ok = motor.move_position(angle_deg=30, speed_rpm=150, mode="direct")
                _require(ok, f"move on {fw} failed")
                motor.wait_position_reached(timeout=6)
                notes.append(fw.name)
                motor.stop()
            motor.switch_firmware(original, store=True)
            motor.disable()
            return ",".join(notes)

        out.append(_run("move_on_both_firmware", _move_both))

    return out


def test_homing(motor: Y42Device, flags: Flags) -> List[CaseResult]:
    if not flags.home:
        return [_skip("home", "加 --home")]

    out: List[CaseResult] = []

    def _hp() -> str:
        p = motor.get_homing_params()
        return f"mode={p.homing_mode} speed={p.homing_speed}"

    out.append(_run("get_homing_params", _hp))

    def _home() -> str:
        motor.enable()
        ok = motor.home(mode=HomingMode.NEAREST)
        st = motor.wait_homing(timeout=15)
        motor.disable()
        return f"cmd={ok} wait={st}"

    out.append(_run("home_nearest", _home))
    return out


def test_timed(motor: Y42Device, flags: Flags) -> List[CaseResult]:
    if not flags.timed:
        return [_skip("timed_return", "加 --timed")]

    def _tr() -> str:
        _require(
            motor.timed_return(Code.GET_REALTIME_POSITION, interval_ms=100),
            "start failed",
        )
        time.sleep(0.35)
        _require(motor.timed_return(Code.GET_REALTIME_POSITION, interval_ms=0), "stop failed")
        return "ok"

    return [_run("timed_return", _tr)]


def test_misc(motor: Y42Device, flags: Flags) -> List[CaseResult]:
    out: List[CaseResult] = []
    if flags.calibrate:

        def _cal() -> str:
            _require(motor.calibrate_encoder(), "calibrate failed")
            return "ok"

        out.append(_run("calibrate_encoder", _cal))
    else:
        out.append(_skip("calibrate_encoder", "加 --calibrate"))

    if flags.restart:

        def _rst() -> str:
            _require(motor.restart(), "restart failed")
            return "ok"

        out.append(_run("restart", _rst))
    else:
        out.append(_skip("restart", "加 --restart"))
    return out


def test_dual(ser: Serial, addrs: List[int], flags: Flags) -> List[CaseResult]:
    if not flags.dual or len(addrs) < 2:
        return [_skip("dual", "需要 --dual 且至少两个地址")]

    m1 = Y42Device(ser, address=addrs[0], auto_test=False, firmware_type=None)
    m2 = Y42Device(ser, address=addrs[1], auto_test=False, firmware_type=None)
    m1.detect_firmware()
    m2.detect_firmware()

    def _sync() -> str:
        m1.enable(sync=True)
        m2.enable(sync=True)
        m1.jog(speed_rpm=40, sync=True)
        m2.jog(speed_rpm=40, direction=Direction.CCW, sync=True)
        _require(Y42Device.sync_move(m1.device_params), "sync_move failed")
        time.sleep(0.4)
        m1.stop()
        m2.stop()
        m1.disable()
        m2.disable()
        return f"addr {addrs[0]}+{addrs[1]}"

    return [_run("sync_move_dual", _sync)]


def run_motor(motor: Y42Device, flags: Flags) -> List[CaseResult]:
    results: List[CaseResult] = []
    results.extend(test_detect(motor))
    results.extend(test_reads(motor))
    results.extend(test_move(motor, flags))
    results.extend(test_firmware_switch(motor, flags))
    results.extend(test_homing(motor, flags))
    results.extend(test_timed(motor, flags))
    results.extend(test_misc(motor, flags))
    return results


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description="Y42 stepper full smoke test")
    p.add_argument("port", nargs="?", default="COM3")
    p.add_argument("--addrs", default="1", help="逗号分隔地址, 如 1,2")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--move", action="store_true")
    p.add_argument("--write", action="store_true")
    p.add_argument("--timed", action="store_true")
    p.add_argument("--home", action="store_true")
    p.add_argument("--firmware", action="store_true", help="固件切换往返(不存储)")
    p.add_argument("--calibrate", action="store_true")
    p.add_argument("--restart", action="store_true")
    p.add_argument("--dual", action="store_true")
    p.add_argument(
        "--full-safe",
        action="store_true",
        help="--move --timed --home --dual（不含 --firmware）",
    )
    args = p.parse_args(argv)

    flags = Flags(
        write=args.write or args.full_safe,
        move=args.move or args.full_safe,
        timed=args.timed or args.full_safe,
        home=args.home or args.full_safe,
        firmware=args.firmware,
        calibrate=args.calibrate,
        restart=args.restart,
        dual=args.dual or args.full_safe,
    )
    addrs = [int(x) for x in args.addrs.split(",") if x.strip()]

    print(f"port={args.port} baud={args.baud} addrs={addrs} flags={flags}")
    ser = Serial(args.port, args.baud, timeout=0.15)
    all_results: List[CaseResult] = []
    try:
        for addr in addrs:
            print(f"\n=== address {addr} ===")
            motor = Y42Device(ser, address=addr)
            rs = run_motor(motor, flags)
            all_results.extend(rs)
            for r in rs:
                mark = "SKIP" if r.skipped else ("PASS" if r.ok else "FAIL")
                print(f"  [{mark}] {r.name}: {r.detail}")

        all_results.extend(test_dual(ser, addrs, flags))
        for r in all_results[-1:]:
            if flags.dual and len(addrs) >= 2:
                mark = "SKIP" if r.skipped else ("PASS" if r.ok else "FAIL")
                print(f"  [{mark}] {r.name}: {r.detail}")
    except Exception:
        traceback.print_exc()
        return 2
    finally:
        ser.close()

    failed = [r for r in all_results if not r.ok]
    skipped = sum(1 for r in all_results if r.skipped)
    print(
        f"\nsummary: total={len(all_results)} "
        f"pass={len(all_results) - len(failed) - skipped} "
        f"fail={len(failed)} skip={skipped}"
    )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
