# 传感器接口开放文档

本文档公开水下海参检测及吸捕机器人项目中的传感器硬件接口、软件驱动入口、配置节点和运行时数据字段。所有端口、引脚、I2C bus、串口和摄像头索引均以 `config/hardware.yaml` 为准，业务逻辑不硬编码硬件连接。

RDK X5 接线核对表、实机检查命令和运行步骤见 [rdk_x5_wiring_and_run.md](rdk_x5_wiring_and_run.md)。

## 总览

| 设备 | 数量 | 物理/通信接口 | 配置节点 | 驱动文件 | 主要输出字段 |
| --- | --- | --- | --- | --- | --- |
| MS5837-30BA 压力/深度传感器 | 1 | 32/33 复用 I2C | `rdk_x5.i2c.ms5837_30ba` | `src/sea_cucumber_robot/sensors/ms5837.py` | `pressure_mbar`, `temperature_c`, `depth_m` |
| VEML7700 光照传感器 | 2 | I2C | `rdk_x5.i2c.veml7700_1`, `rdk_x5.i2c.veml7700_2` | `src/sea_cucumber_robot/sensors/veml7700.py` | `lux`, `als_raw`, `white_raw` |
| DS18B20 温度传感器 | 2 | 1-Wire sysfs | `rdk_x5.one_wire.ds18b20_1`, `rdk_x5.one_wire.ds18b20_2` | `src/sea_cucumber_robot/sensors/ds18b20.py` | `temperature_c` |
| LO81MTW 水下超声波传感器 | 2 | USB 串口 | `ultrasonic_usb.front`, `ultrasonic_usb.downward` | `src/sea_cucumber_robot/sensors/ultrasonic_usb.py` | `distance_m`, `protocol` |
| USB 摄像头 | 2 | USB / V4L2 / OpenCV | `cameras.camera_1`, `cameras.camera_2` | `src/sea_cucumber_robot/vision/camera_manager.py` | `image`, `ok`, `message` |

## 统一软件接口

所有普通传感器驱动继承 `BaseSensor`：

```python
class BaseSensor:
    def open(self) -> None: ...
    def read(self) -> SensorReading: ...
    def close(self) -> None: ...
```

传感器读数统一封装为：

```python
SensorReading(
    name="sensor_name",
    ok=True,
    timestamp_s=...,
    values={...},
    message="",
)
```

统一管理入口：

- `SensorManager.open_all()`：打开配置中启用的传感器。
- `SensorManager.read_all()`：读取所有传感器并返回 `SensorSnapshot`。
- `SensorSnapshot.value(name, key, default)`：读取某个传感器字段。
- `SensorSnapshot.front_distance_m`：读取前向 LO81MTW 的距离值。

硬件检查脚本：

```bash
cd sea_cucumber_robot
python3 scripts/check_sensors.py
python3 scripts/check_sensors.py --simulate
```

## MS5837-30BA 压力/深度传感器

硬件连接：

| 信号 | RDK_X5 40Pin BOARD 物理引脚 |
| --- | --- |
| SDA | PIN33 / 复用 I2C SDA |
| SCL | PIN32 / 复用 I2C SCL |

按提供的 RDK X5 40Pin 对照表，BOARD 物理引脚 32/33 是可复用引脚。它们在默认功能中可能显示为 PWM/GPIO，但复用功能可以切到 I2C，因此 MS5837-30BA 可按 32/33 作为 I2C 传感器接入。

注意事项：

- 需要确认系统 pinmux/设备树已把 PIN32/PIN33 配置为 I2C 功能。
- Linux 下实际 bus 号可能随系统镜像变化，默认配置写为 `bus: 1`，实机需用 `ls /dev/i2c-*` 和 `i2cdetect -y <bus>` 验证。
- 如果 32/33 仍保持 PWM/GPIO 默认功能，`smbus2` 无法直接访问 MS5837。

RDK X5 40Pin 上本项目使用的 I2C 接口：

| 设备 | SDA | SCL | 默认 Linux bus | 说明 |
| --- | --- | --- | --- | --- |
| VEML7700_1 | PIN3 | PIN5 | `5` | I2C5 |
| VEML7700_2 | PIN27 | PIN28 | `0` | I2C0 |
| MS5837-30BA | PIN33 | PIN32 | `1` | 32/33 复用 I2C，需 pinmux |

配置节点：

```yaml
rdk_x5:
  i2c:
    ms5837_30ba:
      enabled: true
      name: ms5837_depth
      bus: 1
      address: 0x76
      alternate_address: 0x77
      sda_pin: 33
      scl_pin: 32
      pinmux_required: true
      pinmux_function: i2c
      model: MS5837-30BA
      fluid_density_kg_m3: 1029.0
```

驱动说明：

- 使用 `smbus2` 访问 I2C。
- 初始化时读取 PROM 系数 C0-C7。
- `open()` 时会读取一次当前压力作为水面/启动基准压力。
- 深度换算使用 `fluid_density_kg_m3` 和重力加速度计算。

输出字段：

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `pressure_mbar` | mbar | 绝对压力 |
| `temperature_c` | 摄氏度 | 传感器温度 |
| `depth_m` | m | 相对启动基准压力估算深度 |
| `raw_d1` | 原始值 | 压力 ADC 原始值 |
| `raw_d2` | 原始值 | 温度 ADC 原始值 |

## VEML7700 光照传感器

硬件连接：

| 设备 | SDA | SCL | 默认 I2C 地址 |
| --- | --- | --- | --- |
| VEML7700_1 | PIN3 | PIN5 | `0x10` |
| VEML7700_2 | PIN27 | PIN28 | `0x10` |

配置节点：

```yaml
rdk_x5:
  i2c:
    veml7700_1:
      enabled: true
      bus: 5
      address: 0x10
      sda_pin: 3
      scl_pin: 5
    veml7700_2:
      enabled: true
      bus: 0
      address: 0x10
      sda_pin: 27
      scl_pin: 28
```

输出字段：

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `lux` | lux | 光照强度，默认按 gain x1、100 ms 积分时间换算 |
| `als_raw` | count | ALS 原始值 |
| `white_raw` | count | WHITE 通道原始值 |

## DS18B20 温度传感器

硬件连接：

| 设备 | DATA | GPIO | 说明 |
| --- | --- | --- | --- |
| DS18B20_1 | PIN11 | GPIO17 | 默认温度传感器 1 数据线 |
| DS18B20_2 | PIN13 | GPIO27 | 默认温度传感器 2 数据线 |

DS18B20_2 的原始接线记录存在歧义：

- 可能是 PIN17 作为供电。
- 可能是 PIN13(GPIO27) 作为 DATA。
- 项目配置保留 `power_pin: 17` 和 `data_pin: 13 / gpio: 27`。
- 代码默认使用 GPIO27 作为 DS18B20_2 的 DATA 线，但实机前必须人工确认。

配置节点：

```yaml
rdk_x5:
  one_wire:
    ds18b20_1:
      data_pin: 11
      gpio: 17
      device_id: null
      sysfs_root: /sys/bus/w1/devices
    ds18b20_2:
      power_pin: 17
      data_pin: 13
      gpio: 27
      device_id: null
      sysfs_root: /sys/bus/w1/devices
```

驱动说明：

- 使用 Linux 1-Wire sysfs：`/sys/bus/w1/devices/28-*/w1_slave`。
- 如果配置 `device_id`，驱动会读取指定设备。
- 如果未配置 `device_id`，驱动默认读取扫描到的第一个 `28-*` 设备。

输出字段：

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `temperature_c` | 摄氏度 | DS18B20 温度值 |

## LO81MTW 水下超声波传感器

硬件连接：

| 设备 | 接入方式 | 默认端口 | 角色 |
| --- | --- | --- | --- |
| 传感器 1 | USB 串口 | `/dev/ttyUSB0` | 前向，测量海参与吸捕口距离 |
| 传感器 2 | USB 串口 | `/dev/ttyUSB1` | 向下，测量离底距离或下潜高度 |

配置节点：

```yaml
ultrasonic_usb:
  front:
    port: /dev/ttyUSB0
    baudrate: 9600
    protocol: ff_uart
    min_valid_m: 0.03
    max_valid_m: 4.5
    near_range_fault_below_m: 0.04
  downward:
    port: /dev/ttyUSB1
    baudrate: 9600
    protocol: ff_uart
```

已开放协议：

| 协议 | 帧格式 | 说明 |
| --- | --- | --- |
| `ff_uart` | `FF Data_H Data_L SUM` | 官方 Arduino 示例中的 4 字节 UART 帧，距离单位为 mm |
| `modbus` | `01 03 01 01 00 01 D4 36` | 官方 Arduino 示例中的实时距离读取命令 |

驱动函数：

- `parse_ff_uart_frame(raw)`：解析 FF UART 距离帧。
- `modbus_crc16(data)`：计算 Modbus CRC16。
- `append_modbus_crc(payload)`：追加 Modbus CRC。
- `parse_modbus_distance(frame, address)`：解析 Modbus 距离响应。

输出字段：

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `distance_m` | m | 距离值 |
| `protocol` | - | 当前使用协议 |

状态机注意事项：

- 前向超声用于 `APPROACH_TO_5_5CM`，目标距离为 `0.055 m`。
- 切换到摄像头 2 后，前向超声可能低于量程并返回错误值。
- `SWITCH_TO_CAMERA_2` 阶段不会因为前向超声近距离错误值立即退出任务。

## USB 摄像头接口

硬件连接：

| 摄像头 | 默认设备索引 | 角色 | 默认状态 |
| --- | --- | --- | --- |
| 摄像头 1 | `0` | 前向摄像头 | 开启 |
| 摄像头 2 | `1` | 吸捕口近距离摄像头 | 关闭 |

配置节点：

```yaml
cameras:
  camera_1:
    role: front
    device: 0
    width: 1280
    height: 720
    fps: 30
    default_open: true
  camera_2:
    role: suction_mouth
    device: 1
    width: 1280
    height: 720
    fps: 30
    default_open: false
```

软件接口：

```python
CameraManager.initialize_defaults()
CameraManager.open_camera("camera_1")
CameraManager.close_camera("camera_1")
CameraManager.switch_to("camera_2")
CameraManager.read("camera_2")
```

摄像头帧封装：

```python
CameraFrame(
    camera_id="camera_1",
    image=frame,
    ok=True,
    message="",
)
```

## 调试建议

1. 先用 `python3 scripts/check_sensors.py --simulate` 验证软件接口。
2. 再接入单个传感器，逐项修改 `config/hardware.yaml`。
3. 用 `ls /dev/i2c-*` 和 `i2cdetect -y <bus>` 校验 I2C bus。
4. 用 `ls /dev/ttyUSB*` 固定 LO81MTW 的 USB 串口映射。
5. 用 `python3 scripts/check_cameras.py` 校验两个 USB 摄像头索引。
6. 实机运行前，先确认所有传感器 `ok=True` 且输出字段合理。
