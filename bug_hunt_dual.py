"""Y42 双电机全方位工况压测 / Bug 狩猎.

覆盖: 只读探测、使能停机、速度/位置/力矩、同步与多机、固件切换、
地址隔离、急停打断、快速连发、边界参数、回零(就近)、异常路径。

用法:
  python bug_hunt_dual.py [COM口]
  python bug_hunt_dual.py COM9
"""

from __future__ import annotations

import sys
import time
import traceback
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from serial import Serial

from y42_stepper import (
    Direction,
    FirmwareType,
    HomingMode,
    MotionMode,
    Y42Device,
)
from y42_stepper.commands import (
    EmmJog,
    EmmPosition,
    FirmwareCapabilityError,
    XJog,
    XPositionDirect,
    build_command_frame,
)
from y42_stepper.configs import Code, Protocol, SyncFlag, add_checksum
from y42_stepper.parameters import (
    EmmJogParams,
    EmmPositionParams,
    MAX_SPEED_RPM,
    XJogParams,
    XPositionDirectParams,
)


@dataclass
class Case:
    name: str
    ok: bool
    detail: str = ""
    severity: str = "bug"  # bug | warn | info


@dataclass
class Suite:
    cases: List[Case] = field(default_factory=list)

    def add(self, name: str, ok: bool, detail: str = "", severity: str = "bug") -> bool:
        mark = "PASS" if ok else ("WARN" if severity == "warn" else "FAIL")
        extra = f" | {detail}" if detail else ""
        print(f"  [{mark}] {name}{extra}")
        self.cases.append(Case(name, ok, detail, severity))
        return ok

    def run(self, name: str, fn: Callable[[], str], severity: str = "bug") -> bool:
        try:
            detail = fn() or ""
            return self.add(name, True, detail, severity)
        except Exception as e:
            return self.add(
                name, False, f"{type(e).__name__}: {e}", severity
            )


def section(title: str) -> None:
    print(f"\n{'=' * 64}\n{title}\n{'=' * 64}")


def settle(sec: float = 0.15) -> None:
    time.sleep(sec)


def safe_stop_disable(*motors: Y42Device) -> None:
    for m in motors:
        try:
            m.stop()
        except Exception:
            pass
    settle(0.08)
    for m in motors:
        try:
            m.disable()
        except Exception:
            pass


def switch_fw(m: Y42Device, fw: FirmwareType, tag: str, S: Suite) -> bool:
    m.stop()
    settle(0.05)
    ok = m.switch_firmware(fw, store=True)
    actual = m.detect_firmware()
    return S.add(
        f"{tag}_switch_{fw.name}",
        ok and actual == fw,
        f"ok={ok} actual={actual.name}",
    )


def switch_both(m1: Y42Device, m2: Y42Device, fw: FirmwareType, S: Suite) -> bool:
    ok1 = switch_fw(m1, fw, "m1", S)
    ok2 = switch_fw(m2, fw, "m2", S)
    return ok1 and ok2


# ---------------------------------------------------------------------------
# 只读 / 基线
# ---------------------------------------------------------------------------

def test_baseline(m1: Y42Device, m2: Y42Device, S: Suite) -> None:
    section("A. 基线探测 (双轴)")

    for tag, m in (("m1", m1), ("m2", m2)):
        def _ver(mm=m) -> str:
            v = mm.get_version()
            return f"{v.firmware_version_str} hw={v.hw_type_str}"

        S.run(f"{tag}_version", _ver)

        def _fw(mm=m) -> str:
            fw = mm.detect_firmware()
            opt = mm.get_option_status()
            mismatch = (opt.firmware_is_emm != (fw == FirmwareType.EMM_FIRMWARE))
            detail = f"{fw.name} opt_bit_emm={opt.firmware_is_emm}"
            if mismatch:
                # 已知问题：0x1A 不可靠
                S.add(
                    f"{tag}_opt_fw_bit_vs_detect",
                    False,
                    detail + " (已知: 0x1A 不可靠)",
                    severity="warn",
                )
            return detail

        S.run(f"{tag}_detect_fw", _fw)

        def _reads(mm=m) -> str:
            vbus = mm.get_bus_voltage()
            temp = mm.get_temperature()
            enc = mm.get_encoder()
            spd = mm.get_realtime_speed()
            pos = mm.get_realtime_position()
            st = mm.get_motor_status()
            cfg = mm.get_config()
            pid = mm.get_pid()
            sys = mm.get_system_status()
            assert vbus > 5000, f"vbus too low: {vbus}"
            return (
                f"V={vbus/1000:.2f} T={temp} enc={enc} spd={spd} "
                f"pos={pos:.2f} en={st.enabled} cfg={type(cfg).__name__} "
                f"pid={type(pid).__name__} sysV={sys.bus_voltage}"
            )

        S.run(f"{tag}_reads", _reads)

    # 地址应不同且各自应答
    def _addr_ok() -> str:
        assert m1.address == 1 and m2.address == 2
        v1 = m1.get_version()
        v2 = m2.get_version()
        return f"v1={v1.firmware_version_str} v2={v2.firmware_version_str}"

    S.run("addr_identity", _addr_ok)


# ---------------------------------------------------------------------------
# 单轴运动矩阵
# ---------------------------------------------------------------------------

def test_motion_single(m: Y42Device, tag: str, fw: FirmwareType, S: Suite) -> None:
    section(f"B. 单轴运动 {tag} / {fw.name}")

    def _enable() -> str:
        ok = m.enable()
        settle(0.1)
        st = m.get_motor_status()
        assert ok and st.enabled, f"ok={ok} en={st.enabled}"
        return "enabled"

    S.run(f"{tag}_enable", _enable)

    # --- jog CW/CCW ---
    for dir_, dname in ((Direction.CW, "CW"), (Direction.CCW, "CCW")):
        def _jog(d=dir_, dn=dname) -> str:
            accel = 0 if fw == FirmwareType.EMM_FIRMWARE else 2000
            ok = m.jog(speed_rpm=120, direction=d, acceleration=accel)
            settle(0.5)
            spd = m.get_realtime_speed()
            m.stop()
            settle(0.2)
            # 方向: CW 通常为正，但板卡符号约定可能相反，只要求有速度
            assert ok and abs(spd) > 30, f"ok={ok} spd={spd}"
            return f"spd={spd:.1f}"

        S.run(f"{tag}_jog_{dname}", _jog)

    # --- 速度跟踪阶梯 ---
    def _ladder() -> str:
        accel = 0 if fw == FirmwareType.EMM_FIRMWARE else 5000
        peaks = []
        for rpm in (50, 200, 500, 1000, 2000):
            ok = m.jog(speed_rpm=rpm, direction=Direction.CW, acceleration=accel)
            settle(0.8)
            samples = [abs(m.get_realtime_speed()) for _ in range(4)]
            peak = max(samples)
            peaks.append((rpm, peak, ok))
            m.stop()
            settle(0.25)
            # 允许 ±15% 或 ±30rpm
            tol = max(30, rpm * 0.15)
            assert ok and abs(peak - rpm) <= tol, f"cmd={rpm} peak={peak}"
        return str([(c, round(p)) for c, p, _ in peaks])

    S.run(f"{tag}_speed_ladder", _ladder)

    # --- 高转速 (库上限内) ---
    def _high() -> str:
        accel = 0 if fw == FirmwareType.EMM_FIRMWARE else 8000
        target = 3000
        ok = m.jog(speed_rpm=target, direction=Direction.CW, acceleration=accel)
        settle(1.2)
        peak = max(abs(m.get_realtime_speed()) for _ in range(5))
        for _ in range(5):
            time.sleep(0.05)
            peak = max(peak, abs(m.get_realtime_speed()))
        m.stop()
        settle(0.3)
        # 高转速允许更大偏差
        assert ok and peak > target * 0.7, f"ok={ok} peak={peak}"
        return f"peak={peak:.1f}"

    S.run(f"{tag}_jog_3000", _high)

    # --- 超手册但库内允许 ---
    def _over_manual() -> str:
        accel = 0 if fw == FirmwareType.EMM_FIRMWARE else 8000
        ok = m.jog(speed_rpm=4000, direction=Direction.CW, acceleration=accel)
        settle(1.0)
        peak = max(abs(m.get_realtime_speed()) for _ in range(6))
        for _ in range(5):
            time.sleep(0.05)
            peak = max(peak, abs(m.get_realtime_speed()))
        m.stop()
        settle(0.3)
        # 不强制达到 4000（受供电限制），但命令应成功且有高速
        assert ok and peak > 2000, f"ok={ok} peak={peak}"
        return f"peak={peak:.1f} (cmd=4000)"

    S.run(f"{tag}_jog_4000", _over_manual)

    # --- 库上限拒绝 ---
    def _reject() -> str:
        try:
            m.jog(speed_rpm=MAX_SPEED_RPM + 1)
            raise AssertionError("should reject")
        except ValueError as e:
            return str(e)

    S.run(f"{tag}_reject_over_lib_max", _reject)

    # --- 相对位置 ---
    def _pos_rel() -> str:
        p0 = m.get_realtime_position()
        if fw == FirmwareType.EMM_FIRMWARE:
            ok = m.move_position(
                pulses=1600, speed_rpm=300, acceleration=20,
                motion_mode=MotionMode.RELATIVE_CURRENT,
            )
        else:
            ok = m.move_position(
                angle_deg=90, speed_rpm=300, acceleration=1500,
                mode="direct", motion_mode=MotionMode.RELATIVE_CURRENT,
            )
        reached = m.wait_position_reached(timeout=10)
        p1 = m.get_realtime_position()
        assert ok and reached, f"ok={ok} reached={reached}"
        return f"p0={p0:.2f} p1={p1:.2f} d={p1-p0:.2f}"

    S.run(f"{tag}_pos_relative", _pos_rel)

    # --- 绝对位置 (先清零) ---
    def _pos_abs() -> str:
        okz = m.zero_position()
        settle(0.1)
        if fw == FirmwareType.EMM_FIRMWARE:
            # Emm 用脉冲绝对 — 依赖 microstep；用相对当前更稳，这里测绝对小角度
            ok = m.move_position(
                pulses=800, speed_rpm=250, acceleration=15,
                motion_mode=MotionMode.ABSOLUTE,
            )
        else:
            ok = m.move_position(
                angle_deg=45, speed_rpm=250, acceleration=1200,
                mode="direct", motion_mode=MotionMode.ABSOLUTE,
            )
        reached = m.wait_position_reached(timeout=10)
        err = abs(m.get_position_error())
        assert okz and ok and reached, f"z={okz} ok={ok} r={reached}"
        return f"err={err:.3f} pos={m.get_realtime_position():.2f}"

    S.run(f"{tag}_pos_absolute", _pos_abs)

    # --- X 梯形 / Emm 再发相对 ---
    if fw == FirmwareType.X_FIRMWARE:
        def _trap() -> str:
            ok = m.move_position(
                angle_deg=60, speed_rpm=200, acceleration=800, decel=800,
                mode="trap", motion_mode=MotionMode.RELATIVE_CURRENT,
            )
            reached = m.wait_position_reached(timeout=10)
            assert ok and reached
            return "trap ok"

        S.run(f"{tag}_pos_trap", _trap)

        def _torque() -> str:
            # 短时力矩，电流较小
            ok = m.torque(current_ma=400, slope_ma_s=2000, direction=Direction.CW)
            settle(0.4)
            spd = abs(m.get_realtime_speed())
            m.stop()
            settle(0.2)
            assert ok and spd > 5, f"ok={ok} spd={spd}"
            return f"spd={spd:.1f}"

        S.run(f"{tag}_torque", _torque)

        def _torque_lim() -> str:
            ok = m.torque_limited(
                current_ma=500, slope_ma_s=2000,
                max_speed_rpm=80, direction=Direction.CCW,
            )
            settle(0.5)
            spd = abs(m.get_realtime_speed())
            m.stop()
            settle(0.2)
            assert ok and 20 < spd < 200, f"ok={ok} spd={spd}"
            return f"spd={spd:.1f}"

        S.run(f"{tag}_torque_limited", _torque_lim)
    else:
        def _emm_no_torque() -> str:
            try:
                m.torque(current_ma=300)
                raise AssertionError("Emm should reject torque")
            except FirmwareCapabilityError as e:
                return str(e)

        S.run(f"{tag}_emm_reject_torque", _emm_no_torque)

        # Emm 位置打断：运动中发新位置
        def _interrupt() -> str:
            ok1 = m.move_position(
                pulses=8000, speed_rpm=400, acceleration=10,
                motion_mode=MotionMode.RELATIVE_CURRENT,
            )
            settle(0.25)
            ok2 = m.move_position(
                pulses=400, speed_rpm=500, acceleration=20,
                motion_mode=MotionMode.RELATIVE_CURRENT,
            )
            reached = m.wait_position_reached(timeout=8)
            assert ok1 and ok2 and reached
            return "interrupt ok"

        S.run(f"{tag}_emm_pos_interrupt", _interrupt)

    # --- 运动中急停 ---
    def _estop() -> str:
        accel = 0 if fw == FirmwareType.EMM_FIRMWARE else 3000
        m.jog(speed_rpm=800, direction=Direction.CW, acceleration=accel)
        settle(0.4)
        ok = m.stop()
        settle(0.35)
        spd = abs(m.get_realtime_speed())
        assert ok and spd < 40, f"ok={ok} spd={spd}"
        return f"after_stop spd={spd:.1f}"

    S.run(f"{tag}_estop_while_jog", _estop)

    # --- 运动中失能 ---
    def _dis_while() -> str:
        accel = 0 if fw == FirmwareType.EMM_FIRMWARE else 3000
        m.jog(speed_rpm=500, direction=Direction.CW, acceleration=accel)
        settle(0.3)
        ok = m.disable()
        settle(0.35)
        st = m.get_motor_status()
        spd = abs(m.get_realtime_speed())
        m.enable()
        settle(0.1)
        assert ok and not st.enabled and spd < 80, f"en={st.enabled} spd={spd}"
        return f"spd={spd:.1f}"

    S.run(f"{tag}_disable_while_jog", _dis_while)

    # --- 清零 / 读误差 ---
    def _zero() -> str:
        ok = m.zero_position()
        settle(0.05)
        pos = m.get_realtime_position()
        # 清零后应接近 0（允许残余）
        assert ok and abs(pos) < 5.0, f"pos={pos}"
        return f"pos={pos:.3f}"

    S.run(f"{tag}_zero_position", _zero)

    # --- 回零: 设零点 → 离开 → 就近/绝对零 ---
    def _home_near() -> str:
        m.enable()
        okz = m.set_home_zero(store=True)
        settle(0.1)
        if fw == FirmwareType.EMM_FIRMWARE:
            m.move_position(pulses=500, speed_rpm=120, acceleration=12)
        else:
            m.move_position(
                angle_deg=40, speed_rpm=120, acceleration=1000, mode="direct"
            )
        m.wait_position_reached(timeout=8)
        ok = m.home(mode=HomingMode.NEAREST)
        # 若已在单圈零点会回 0x12；再试绝对坐标零点
        if not ok:
            ok = m.home(mode=HomingMode.ABS_ZERO)
        done = m.wait_homing(timeout=15)
        hs = m.get_homing_status()
        assert okz and ok and done and not hs.homing_failed, (
            f"z={okz} ok={ok} done={done} hs={hs}"
        )
        return f"calibrated={hs.calibrated} pos={m.get_realtime_position():.2f}"

    S.run(f"{tag}_home_nearest", _home_near)

    # --- 连续快速 jog 切换方向 ---
    def _dir_flip() -> str:
        accel = 0 if fw == FirmwareType.EMM_FIRMWARE else 4000
        for i in range(8):
            d = Direction.CW if i % 2 == 0 else Direction.CCW
            assert m.jog(speed_rpm=300, direction=d, acceleration=accel)
            settle(0.15)
        m.stop()
        settle(0.25)
        return "8 flips ok"

    S.run(f"{tag}_rapid_dir_flip", _dir_flip)

    # --- 读接口在运动中 ---
    def _read_while_move() -> str:
        accel = 0 if fw == FirmwareType.EMM_FIRMWARE else 2000
        m.jog(speed_rpm=200, direction=Direction.CW, acceleration=accel)
        settle(0.2)
        vals = []
        for _ in range(10):
            vals.append(
                (
                    m.get_realtime_speed(),
                    m.get_realtime_position(),
                    m.get_phase_current(),
                    m.get_bus_voltage(),
                )
            )
            time.sleep(0.05)
        m.stop()
        settle(0.2)
        assert all(v[3] > 5000 for v in vals)
        return f"n={len(vals)} last_spd={vals[-1][0]:.1f}"

    S.run(f"{tag}_read_while_moving", _read_while_move)

    safe_stop_disable(m)


# ---------------------------------------------------------------------------
# 双轴协同
# ---------------------------------------------------------------------------

def test_dual(m1: Y42Device, m2: Y42Device, fw: FirmwareType, S: Suite) -> None:
    section(f"C. 双轴协同 / {fw.name}")
    tag = fw.name
    accel = 0 if fw == FirmwareType.EMM_FIRMWARE else 2000

    m1.enable()
    m2.enable()
    settle(0.1)

    # 地址隔离: 只动 m1
    def _iso() -> str:
        p2a = m2.get_realtime_position()
        m1.jog(speed_rpm=150, direction=Direction.CW, acceleration=accel)
        settle(0.6)
        s1 = abs(m1.get_realtime_speed())
        s2 = abs(m2.get_realtime_speed())
        p2b = m2.get_realtime_position()
        m1.stop()
        settle(0.2)
        assert s1 > 40 and s2 < 20 and abs(p2b - p2a) < 3.0, (
            f"s1={s1} s2={s2} dp2={p2b-p2a}"
        )
        return f"s1={s1:.1f} s2={s2:.1f} dp2={p2b-p2a:.2f}"

    S.run(f"{tag}_addr_isolation", _iso)

    # 对转 jog
    def _opp() -> str:
        ok1 = m1.jog(speed_rpm=100, direction=Direction.CW, acceleration=accel)
        ok2 = m2.jog(speed_rpm=100, direction=Direction.CCW, acceleration=accel)
        settle(0.5)
        s1, s2 = m1.get_realtime_speed(), m2.get_realtime_speed()
        m1.stop(); m2.stop(); settle(0.2)
        assert ok1 and ok2 and abs(s1) > 30 and abs(s2) > 30
        return f"s1={s1:.1f} s2={s2:.1f}"

    S.run(f"{tag}_opposite_jog", _opp)

    # sync 缓存 + FF66
    def _sync() -> str:
        ok1 = m1.jog(speed_rpm=80, direction=Direction.CW, acceleration=accel or 10, sync=True)
        ok2 = m2.jog(speed_rpm=80, direction=Direction.CCW, acceleration=accel or 10, sync=True)
        settle(0.15)
        pre1, pre2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        ok = Y42Device.sync_move(m1.device_params)
        settle(0.45)
        post1, post2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        m1.stop(); m2.stop(); settle(0.2)
        assert ok1 and ok2 and ok and pre1 < 20 and pre2 < 20 and post1 > 20 and post2 > 20, (
            f"pre={pre1}/{pre2} post={post1}/{post2}"
        )
        return f"pre={pre1:.0f}/{pre2:.0f} post={post1:.0f}/{post2:.0f}"

    S.run(f"{tag}_sync_ff66", _sync)

    # 0xAA 多机
    def _aa() -> str:
        cs = m1.device_params.checksum_mode
        if fw == FirmwareType.EMM_FIRMWARE:
            b1 = bytes([1, Code.JOG]) + EmmJogParams(
                direction=Direction.CW, speed=60, acceleration=10
            ).bytes
            b2 = bytes([2, Code.JOG]) + EmmJogParams(
                direction=Direction.CCW, speed=60, acceleration=10
            ).bytes
        else:
            b1 = bytes([1, Code.JOG]) + XJogParams(
                direction=Direction.CW, acceleration_rpm_s=1000, speed_raw=600
            ).bytes
            b2 = bytes([2, Code.JOG]) + XJogParams(
                direction=Direction.CCW, acceleration_rpm_s=1000, speed_raw=600
            ).bytes
        ok = Y42Device.multi_motor(
            [build_command_frame(b1, cs), build_command_frame(b2, cs)],
            m1.device_params,
            expect_ack=True,
        )
        settle(0.45)
        s1, s2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        stop1 = add_checksum(bytes([1, Code.ESTOP, Protocol.ESTOP, 0]), cs)
        stop2 = add_checksum(bytes([2, Code.ESTOP, Protocol.ESTOP, 0]), cs)
        Y42Device.multi_motor([stop1, stop2], m1.device_params, expect_ack=True)
        settle(0.2)
        assert ok and s1 > 10 and s2 > 10, f"ok={ok} s1={s1} s2={s2}"
        return f"s1={s1:.1f} s2={s2:.1f}"

    S.run(f"{tag}_multi_aa", _aa)

    # 同步位置
    def _sync_pos() -> str:
        if fw == FirmwareType.EMM_FIRMWARE:
            ok1 = m1.move_position(pulses=1000, speed_rpm=200, acceleration=15, sync=True)
            ok2 = m2.move_position(pulses=1000, speed_rpm=200, acceleration=15, sync=True)
        else:
            ok1 = m1.move_position(
                angle_deg=40, speed_rpm=200, acceleration=1000, mode="direct", sync=True
            )
            ok2 = m2.move_position(
                angle_deg=40, speed_rpm=200, acceleration=1000, mode="direct", sync=True
            )
        Y42Device.sync_move(m1.device_params)
        r1 = m1.wait_position_reached(timeout=10)
        r2 = m2.wait_position_reached(timeout=10)
        assert ok1 and ok2 and r1 and r2
        return f"r1={r1} r2={r2}"

    S.run(f"{tag}_sync_position", _sync_pos)

    # 一轴急停不影响另一轴继续? (或至少可独立停)
    def _indep_stop() -> str:
        m1.jog(speed_rpm=120, direction=Direction.CW, acceleration=accel or 10)
        m2.jog(speed_rpm=120, direction=Direction.CW, acceleration=accel or 10)
        settle(0.4)
        m1.stop()
        settle(0.35)
        s1, s2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        m2.stop()
        settle(0.2)
        assert s1 < 40 and s2 > 30, f"s1={s1} s2={s2} (期望仅停 m1)"
        return f"s1={s1:.1f} s2={s2:.1f}"

    S.run(f"{tag}_independent_stop", _indep_stop)

    safe_stop_disable(m1, m2)


# ---------------------------------------------------------------------------
# 固件切换压力 / 错固件命令
# ---------------------------------------------------------------------------

def test_fw_stress(m1: Y42Device, m2: Y42Device, S: Suite) -> None:
    section("D. 固件切换与错固件命令")

    # 快速来回切换单轴
    def _flip_m1() -> str:
        seq = []
        for fw in (
            FirmwareType.X_FIRMWARE,
            FirmwareType.EMM_FIRMWARE,
            FirmwareType.X_FIRMWARE,
            FirmwareType.EMM_FIRMWARE,
        ):
            m1.stop()
            ok = m1.switch_firmware(fw, store=True)
            actual = m1.detect_firmware()
            seq.append((fw.name, ok, actual.name))
            assert ok and actual == fw, str(seq)
            m1.enable()
            accel = 0 if fw == FirmwareType.EMM_FIRMWARE else 1500
            assert m1.jog(speed_rpm=80, direction=Direction.CW, acceleration=accel)
            settle(0.35)
            assert abs(m1.get_realtime_speed()) > 20
            m1.stop()
            settle(0.15)
        return str(seq)

    S.run("m1_fw_flip_x4", _flip_m1)

    # 双轴不同固件共存
    def _mixed() -> str:
        assert m1.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
        assert m2.switch_firmware(FirmwareType.X_FIRMWARE, store=True)
        f1, f2 = m1.detect_firmware(), m2.detect_firmware()
        assert f1 == FirmwareType.EMM_FIRMWARE and f2 == FirmwareType.X_FIRMWARE
        m1.enable(); m2.enable()
        assert m1.jog(speed_rpm=70, acceleration=10)
        assert m2.jog(speed_rpm=70, acceleration=1500)
        settle(0.45)
        s1, s2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        m1.stop(); m2.stop()
        assert s1 > 15 and s2 > 15, f"s1={s1} s2={s2}"
        return f"m1={f1.name} m2={f2.name} s={s1:.0f}/{s2:.0f}"

    S.run("mixed_firmware_dual", _mixed)

    # Emm 下误用 X 直通位置命令帧 — 应 EE 或失败
    def _wrong_cmd() -> str:
        m1.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
        m1.enable()
        # 用底层 X 位置命令打到 Emm
        from y42_stepper.commands import XPositionDirect as XP
        from y42_stepper.parameters import XPositionDirectParams as XPP

        params = XPP(
            direction=Direction.CW,
            speed_raw=1000,
            angle_raw=300,
            motion_mode=MotionMode.RELATIVE_CURRENT,
        )
        cmd = XP(m1.device_params, params=params)
        # is_success False 或抛错都可接受；不应让库崩溃
        ok = cmd.is_success
        m1.stop()
        return f"x_cmd_on_emm is_success={ok}"

    S.run("emm_accepts_or_rejects_x_pos", _wrong_cmd, severity="warn")

    safe_stop_disable(m1, m2)


# ---------------------------------------------------------------------------
# 边界 / 异常 / 通信
# ---------------------------------------------------------------------------

def test_edges(m1: Y42Device, m2: Y42Device, S: Suite) -> None:
    section("E. 边界与异常路径")

    m1.detect_firmware()
    if m1.firmware_type != FirmwareType.EMM_FIRMWARE:
        m1.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
    m1.enable()

    def _jog0() -> str:
        ok = m1.jog(speed_rpm=0, acceleration=0)
        settle(0.2)
        spd = abs(m1.get_realtime_speed())
        return f"ok={ok} spd={spd:.1f}"

    S.run("jog_zero_speed", _jog0, severity="warn")

    def _neg() -> str:
        try:
            m1.jog(speed_rpm=-10)
            raise AssertionError("neg speed should fail")
        except ValueError as e:
            return str(e)

    S.run("reject_negative_speed", _neg)

    def _flood() -> str:
        # 快速连发读，不应抛错
        t0 = time.time()
        n = 40
        for _ in range(n):
            m1.get_realtime_speed()
            m2.get_bus_voltage()
        dt = time.time() - t0
        return f"{n*2} reads in {dt:.2f}s"

    S.run("read_flood", _flood)

    def _cmd_flood_jog() -> str:
        for rpm in range(50, 301, 25):
            assert m1.jog(speed_rpm=rpm, acceleration=0)
            time.sleep(0.03)
        settle(0.4)
        spd = abs(m1.get_realtime_speed())
        m1.stop()
        assert spd > 40
        return f"final_spd~{spd:.0f}"

    S.run("jog_cmd_flood", _cmd_flood_jog)

    def _clear_prot() -> str:
        ok = m1.clear_protection()
        return f"ok={ok}"

    S.run("clear_protection", _clear_prot)

    def _timed() -> str:
        # 开短时定时返回再关
        ok1 = m1.timed_return(0x35, interval_ms=50)  # 速度
        settle(0.2)
        ok2 = m1.timed_return(0x35, interval_ms=0)  # 关闭
        return f"on={ok1} off={ok2}"

    S.run("timed_return_toggle", _timed, severity="warn")

    def _cfg_pid_roundtrip_read() -> str:
        cfg = m1.get_config()
        pid = m1.get_pid()
        # 只读一致性：连续两次
        cfg2 = m1.get_config()
        assert type(cfg) is type(cfg2)
        return f"cfg={type(cfg).__name__} pid_kp?={getattr(pid, 'kp', getattr(pid, 'speed_kp', '?'))}"

    S.run("cfg_pid_stable_read", _cfg_pid_roundtrip_read)

    # 未使能时 jog —— 观察行为
    def _jog_disabled() -> str:
        m1.disable()
        settle(0.1)
        ok = m1.jog(speed_rpm=100, acceleration=0)
        settle(0.35)
        spd = abs(m1.get_realtime_speed())
        st = m1.get_motor_status()
        m1.enable()
        # 多数驱动会拒绝或无转速
        return f"ack={ok} spd={spd:.1f} en={st.enabled}"

    S.run("jog_while_disabled", _jog_disabled, severity="warn")

    safe_stop_disable(m1, m2)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
    print(f"Y42 Bug Hunt — port={port} addrs=1,2  lib_max={MAX_SPEED_RPM}RPM")
    S = Suite()
    ser = Serial(port, 115200, timeout=0.18)
    originals: dict[int, FirmwareType] = {}
    m1 = m2 = None

    try:
        m1 = Y42Device(ser, address=1)
        m2 = Y42Device(ser, address=2)
        originals[1] = m1.detect_firmware()
        originals[2] = m2.detect_firmware()
        vbus = m1.get_bus_voltage()
        section(
            f"初始 m1={originals[1].name} m2={originals[2].name} "
            f"Vbus={vbus/1000:.2f}V"
        )

        test_baseline(m1, m2, S)

        for fw in (FirmwareType.EMM_FIRMWARE, FirmwareType.X_FIRMWARE):
            if not switch_both(m1, m2, fw, S):
                S.add(f"abort_suite_{fw.name}", False, "switch failed")
                continue
            test_motion_single(m1, "m1", fw, S)
            test_motion_single(m2, "m2", fw, S)
            test_dual(m1, m2, fw, S)

        test_fw_stress(m1, m2, S)
        test_edges(m1, m2, S)

    except Exception:
        print("\n[FATAL]\n" + traceback.format_exc())
        S.add("fatal_exception", False, traceback.format_exc()[-200:])
    finally:
        section("恢复原固件并停机")
        try:
            for addr, fw in originals.items():
                m = Y42Device(ser, address=addr, auto_test=False)
                m.detect_firmware()
                m.stop()
                ok = m.switch_firmware(fw, store=True)
                S.add(f"restore_addr{addr}", ok, fw.name, severity="warn")
                m.disable()
        except Exception as e:
            print(f"  [WARN] restore: {e}")
        try:
            ser.close()
        except Exception:
            pass

    section("汇总")
    fails = [c for c in S.cases if not c.ok and c.severity == "bug"]
    warns = [c for c in S.cases if not c.ok and c.severity == "warn"]
    passes = [c for c in S.cases if c.ok]
    print(f"total={len(S.cases)} pass={len(passes)} FAIL={len(fails)} WARN={len(warns)}")
    if fails:
        print("\n--- FAIL (疑似 Bug) ---")
        for c in fails:
            print(f"  - {c.name}: {c.detail}")
    if warns:
        print("\n--- WARN (已知/需人工判断) ---")
        for c in warns:
            print(f"  - {c.name}: {c.detail}")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
