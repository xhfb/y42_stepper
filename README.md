# y42_stepper — ZDT Y42 双固件闭环步进电机控制库

基于 **ZDT Y42 第二代闭环步进电机使用说明 V1.1**，同时支持板载 **X 固件** 与 **Emm 固件**，并封装固件切换（`0xD5`）。

实机验证：硬件 **Y42**，固件 **V2.0.5**（Emm / X 可切换）。

## 参考文档与相关项目

- 协议手册：仓库内 `docs/`（或厂商 PDF《ZDT Y42 第二代闭环步进电机使用说明 V1.1》）
- 架构参考：[xhfb/emm_stepper](https://github.com/xhfb/emm_stepper)（X42S / Emm-only 控制库）

> **与 emm_stepper 的关系**  
> `emm_stepper` 面向 **X42S + Emm 固件**；本库面向 **Y42 主板 + X/Emm 双固件**。  
> 通讯帧风格相近，但配置布局、运动命令载荷、能力集不同，**不能混用设备类**。

## 目录

- [特性](#特性)
- [安装](#安装)
- [快速开始](#快速开始)
- [双固件说明](#双固件说明)
- [API 概览](#api-概览)
- [使用示例](#使用示例)
- [实机测试脚本](#实机测试脚本)
- [注意事项](#注意事项)
- [许可证](#许可证)

## 特性

- ✅ 串口 TTL / RS485 自由协议（校验 FIXED `0x6B` / XOR / CRC8）
- ✅ 同一串口多电机（不同地址）
- ✅ **X / Emm 双固件**统一 API（度 / RPM / mA），底层自动编码
- ✅ 固件探测与切换（`switch_firmware`，建议 `store=True`）
- ✅ 速度模式（库侧允许 0–6000 RPM；见下方实机限速说明）
- ✅ 位置模式（Emm 脉冲；X 直通 / 梯形，单位 0.1°）
- ✅ X 力矩 / 限速力矩 / 限流速度 / 力位混合
- ✅ Emm 位置命令打断（运动中可发新位置）
- ✅ 原点回零（就近 / 方向 / 碰撞 / 限位 / 绝对零点 / 掉电位置）
- ✅ 多机同步 `FF 66`、多电机命令 `0xAA`
- ✅ 定时返回、电池电压、总线电流、温度、IO、心跳等 Y42 增量能力
- ✅ 读取状态（位置、速度、电流、温度、堵转标志等）

## 安装

本项目已发布到 PyPI，推荐直接使用 pip 安装：

```bash
pip install y42_stepper
```

该命令会自动安装依赖（`pyserial>=3.5`）。需要 Python `>=3.8`。

也可从源码或 GitHub 安装：

```bash
cd y42_stepper && pip install -e .
# 或
pip install git+https://github.com/xhfb/y42_stepper.git
```

PyPI：[https://pypi.org/project/y42-stepper/](https://pypi.org/project/y42-stepper/)

### 导入

```python
from serial import Serial
from y42_stepper import (
    Y42Device,
    Direction,
    FirmwareType,
    MotionMode,
    HomingMode,
    ChecksumMode,
)
```

## 快速开始

```python
from serial import Serial
from y42_stepper import Y42Device, Direction, MotionMode

ser = Serial("COM9", 115200, timeout=0.1)
motor = Y42Device(ser, address=1)  # 自动探测当前固件

print(motor.firmware_type)  # X_FIRMWARE 或 EMM_FIRMWARE
print(motor.get_version().firmware_version_str)

motor.enable()

# 速度模式（统一用 RPM）
motor.jog(speed_rpm=100, direction=Direction.CW)
import time
time.sleep(2)
motor.stop()

# 位置模式：可用角度；Emm 也可用 pulses=
motor.move_position(
    angle_deg=90,
    speed_rpm=200,
    motion_mode=MotionMode.RELATIVE_CURRENT,
)
motor.wait_position_reached(timeout=10)

motor.disable()
ser.close()
```

## 双固件说明

| 项目 | X 固件 | Emm 固件 |
|------|--------|----------|
| 速度 `F6` | 加速度单位 RPM/s；速度原始值 0.1 RPM | 加速度 0–255 档位；速度整数 RPM |
| 位置 | 直通 `FB` / 梯形 `FD`（角度 0.1°） | `FD` 脉冲位置 |
| 力矩 | `F5` / 限速 `C5` | 无（抛 `FirmwareCapabilityError`） |
| 限流运动 | `C6` / `CB` / `CD` 等 | 无 |
| 实机有效转速上限（Y42 V2.0.5） | 约 **3000 RPM**（更高指令会 ACK 但被限幅） | 手册标称 3000；较高母线电压下可更高（如 19V 空载可超 4000） |
| 面板指示（手册） | 绿灯约 1s 闪烁 | 绿灯常亮 |

### 固件探测（重要）

**不要信任** `0x1A` 选项状态里的 FwType 位（Y42 V2.0.x 上 Emm 时该位仍可能为 0）。

本库 `detect_firmware()` 通过读取配置块长度判定：

- 字节数 `0x25` → X 固件
- 字节数 `0x21` → Emm 固件

### 固件切换

```python
from y42_stepper import FirmwareType

motor.stop()
# 实机上 store=False 常显示成功但不生效，请用 store=True
ok = motor.switch_firmware(FirmwareType.X_FIRMWARE, store=True)
print(ok, motor.firmware_type)

# 也可按键长按 Next 约 3 秒切换（见厂商手册）
```

二进制 OTA 不在本库范围，见厂商《固件更新使用说明》。

## API 概览

### Y42Device 初始化

```python
Y42Device(
    serial_connection: Serial,   # pyserial Serial
    address: int = 1,            # 1-255
    checksum_mode: ChecksumMode = ChecksumMode.FIXED,
    delay: float | None = None,
    auto_test: bool = True,
    firmware_type: FirmwareType | None = None,  # None=自动探测
)
```

### 运动控制

| 方法 | 说明 |
|------|------|
| `enable()` / `disable()` / `stop()` | 使能 / 失能 / 急停 |
| `jog(speed_rpm, direction, acceleration, sync, max_current_ma)` | 速度模式；X 可带限流 |
| `move_position(angle_deg=..., pulses=..., speed_rpm=..., mode=...)` | 位置模式；X 的 `mode` 为 `"trap"` / `"direct"` |
| `torque(current_ma, ...)` / `torque_limited(...)` | 仅 X |
| `wait_position_reached(timeout)` | 等待到位 |
| `sync_move(device_params)` | 静态：广播触发同步 |
| `multi_motor(frames, device_params)` | 静态：`0xAA` 多机一包 |

```python
# Emm：加速度为档位
motor.jog(speed_rpm=200, acceleration=10)

# X：加速度为 RPM/s
motor.jog(speed_rpm=200, acceleration=1000)

# X 直通位置 90°
motor.move_position(angle_deg=90, speed_rpm=300, mode="direct")

# Emm 脉冲一圈（默认 16 细分、1.8° 电机 ≈ 3200 脉冲）
motor.move_position(pulses=3200, speed_rpm=300, acceleration=20)
```

### 固件与能力

| 方法 / 属性 | 说明 |
|-------------|------|
| `detect_firmware()` | 可靠探测并刷新内部 profile |
| `switch_firmware(fw, store=True)` | 切换并校验 |
| `firmware_type` | 当前固件枚举 |
| `supports_torque` | 是否支持力矩 |
| `supports_position_direct` | 是否支持 X 直通位置 |

### 回零

| 方法 | 说明 |
|------|------|
| `set_home_zero(store=True)` | 设置单圈零点（建议回零前调用） |
| `home(mode, sync)` | 触发回零 |
| `stop_home()` | 中断回零 |
| `wait_homing(timeout)` | 等待回零结束 |
| `get_homing_status()` / `get_homing_params()` / `set_homing_params(...)` | 状态与参数 |

### 读取

常用：`get_version`、`get_bus_voltage`、`get_phase_current`、`get_temperature`、`get_encoder`、`get_realtime_speed`、`get_realtime_position`、`get_position_error`、`get_motor_status`、`get_config`、`get_pid`、`get_system_status`、`get_battery_voltage`、`get_io_status`、`timed_return(info_code, interval_ms)` 等。

### 配置写入

`set_id`、`set_microstep`、`set_loop_mode`、`set_open_loop_current`、`set_closed_loop_current`、`set_pid`、`set_config`、`set_auto_run`、`set_heartbeat_time`、`set_position_window`、`clear_protection`、`zero_position`、`calibrate_encoder`、`restart`、`factory_reset` 等（部分写参依赖当前固件布局）。

## 使用示例

### 多电机（共享串口）

```python
from serial import Serial
from y42_stepper import Y42Device, Direction

ser = Serial("COM9", 115200, timeout=0.1)
m1 = Y42Device(ser, address=1)
m2 = Y42Device(ser, address=2)

m1.enable()
m2.enable()
m1.jog(speed_rpm=80, direction=Direction.CW)
m2.jog(speed_rpm=80, direction=Direction.CCW)

import time
time.sleep(2)
m1.stop()
m2.stop()
m1.disable()
m2.disable()
ser.close()
```

### 同步触发（FF66）

```python
from y42_stepper import Y42Device, Direction

# ... 创建 m1, m2 并 enable ...
m1.jog(speed_rpm=100, direction=Direction.CW, sync=True)
m2.jog(speed_rpm=100, direction=Direction.CCW, sync=True)
Y42Device.sync_move(m1.device_params)  # 两轴齐发
```

### X 力矩模式

```python
if motor.supports_torque:
    motor.enable()
    motor.torque(current_ma=400, slope_ma_s=2000, direction=Direction.CW)
    # 或限速力矩
    motor.torque_limited(
        current_ma=500,
        max_speed_rpm=80,
        slope_ma_s=2000,
        direction=Direction.CCW,
    )
    motor.stop()
```

### 固件切换后再运动

```python
from y42_stepper import FirmwareType

motor.stop()
motor.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
motor.enable()
motor.jog(speed_rpm=150, acceleration=10)  # Emm 档位加速度
motor.stop()
motor.disable()
```

### 回零

```python
from y42_stepper import HomingMode

motor.enable()
motor.set_home_zero(store=True)
motor.move_position(angle_deg=90, speed_rpm=120)
motor.wait_position_reached()
motor.home(mode=HomingMode.NEAREST)
motor.wait_homing(timeout=15)
motor.disable()
```

### 读取状态

```python
v = motor.get_version()
print(v.firmware_version_str, v.hw_type_str)
print("Vbus", motor.get_bus_voltage() / 1000, "V")
print("T", motor.get_temperature(), "C")
print("speed", motor.get_realtime_speed(), "RPM")
print("pos", motor.get_realtime_position(), "deg")
st = motor.get_motor_status()
print("enabled", st.enabled, "stall", st.stall_detected)
```

## 实机测试脚本

仓库根目录提供若干实测脚本（请按串口名修改，默认常为 `COM9`）：

```bash
python full_test.py COM9
python full_test.py COM9 --move --firmware
python dual_fw_control_test.py COM9
python dual_motor_sync_test.py COM9
python bug_hunt_dual.py COM9          # 双轴工况压测
```

## 注意事项

### 通讯

1. 默认波特率 **115200**，超时建议 ≥ 0.1s
2. 校验模式需与电机配置一致（默认 FIXED `0x6B`）
3. 开启 `timed_return` 后总线会有异步帧，关闭前注意清空输入缓冲

### 固件与速度

1. 探测请用 `detect_firmware()`，勿依赖 `get_option_status().firmware_is_emm`
2. 切换固件务必 **停机** 且优先 **`store=True`**
3. 库侧速度校验上限为 **6000 RPM**；X 固件实机有效上限约 **3000 RPM**
4. Emm 在更高母线电压下空载可超过手册标称 3000（属硬件余量，非协议保证）

### 回零

1. 单圈就近回零前建议先 `set_home_zero(store=True)`
2. 碰撞 / 限位回零需按手册接线并设置检测参数
3. 回零应答可能为 `02` / `9F` / `12`（已在零点）等，本库按成功处理可动作应答

### 多电机

1. 每轴独立地址（1–255）
2. `sync=True` 仅缓存，需再调 `Y42Device.sync_move(...)`
3. `factory_reset` / `set_id` / 校准等会改变设备状态，请谨慎

### 错误处理

```python
from y42_stepper import FirmwareCapabilityError

try:
    motor.torque(current_ma=300)  # Emm 下会失败
except FirmwareCapabilityError as e:
    print(e)

if not motor.enable():
    print("使能失败")
if motor.is_stalled():
    motor.clear_protection()
```

## 许可证

MIT License

## 作者

XHFB
