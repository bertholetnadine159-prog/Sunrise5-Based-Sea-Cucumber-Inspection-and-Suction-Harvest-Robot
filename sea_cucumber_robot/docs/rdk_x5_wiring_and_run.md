# RDK X5 接线核对与实机运行指南

本文档用于核对当前工程中所有传感器接线配置，并说明如何在 RDK X5 上逐项检查硬件、安装依赖、运行仿真和启动实机任务。所有端口和引脚以 `config/hardware.yaml` 为准。

## 接线核对结论

| 设备 | 当前配置 | 软件结论 | 实机必须验证 |
| --- | --- | --- | --- |
| VEML7700_1 光照传感器 | SDA `PIN3`，SCL `PIN5`，I2C bus `5`，地址 `0x10` | 与 RDK X5 40Pin I2C5 接口匹配 | `sudo i2cdetect -y 5` 应看到 `0x10` |
| VEML7700_2 光照传感器 | SDA `PIN27`，SCL `PIN28`，I2C bus `0`，地址 `0x10` | 与 RDK X5 40Pin I2C0 接口匹配 | `sudo i2cdetect -y 0` 应看到 `0x10` |
| MS5837-30BA 压力/深度传感器 | SDA `PIN33`，SCL `PIN32`，I2C bus `1`，地址 `0x76`，备用地址 `0x77` | 按当前 40Pin 复用表，`PIN32/PIN33` 可作为复用 I2C 使用，配置已按 32/33 I2C 方案修正 | 必须确认 pinmux/设备树已把 `PIN32/PIN33` 切到 I2C；`sudo i2cdetect -y 1` 应看到 `0x76` 或 `0x77` |
| DS18B20_1 温度传感器 | DATA `PIN11`，GPIO `17` | 配置与 GPIO17 数据线一致 | `/sys/bus/w1/devices/28-*` 中应出现 1-Wire 设备 |
| DS18B20_2 温度传感器 | `power_pin: 17`，DATA `PIN13`，GPIO `27` | 按原始说明保留歧义，代码默认使用 GPIO27 作为 DATA | 人工确认 `PIN17` 是否仅为供电、`PIN13(GPIO27)` 是否为 DATA |
| LO81MTW 前向超声波 | USB 串口 `/dev/ttyUSB0`，`9600` baud，`ff_uart` | 接口配置正确，用于吸捕口前向测距 | 插拔后确认端口是否仍为 `/dev/ttyUSB0` |
| LO81MTW 下向超声波 | USB 串口 `/dev/ttyUSB1`，`9600` baud，`ff_uart` | 接口配置正确，用于离底距离或下潜高度 | 插拔后确认端口是否仍为 `/dev/ttyUSB1` |
| USB 摄像头 1 | OpenCV device `0`，前向摄像头，默认打开 | 配置为默认前向检测摄像头 | `v4l2-ctl --list-devices` 和 `scripts/check_cameras.py` 确认画面方向 |
| USB 摄像头 2 | OpenCV device `1`，吸捕口近距离摄像头，默认关闭 | 配置为到达 5.5 cm 后切换使用 | `v4l2-ctl --list-devices` 和 `scripts/check_cameras.py` 确认索引 |

结论：按当前工程配置，VEML7700、DS18B20、LO81MTW 和 USB 摄像头的软件接线映射是完整的；MS5837-30BA 已按 `PIN32/PIN33` 复用 I2C 方式配置，但它依赖 RDK X5 系统镜像的 pinmux/设备树设置。只要系统中对应 I2C bus 存在，并且 `i2cdetect` 能扫到 `0x76` 或 `0x77`，该接线方案就是可用的。

## RDK X5 系统准备

进入 RDK X5 后先安装基础工具：

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip i2c-tools python3-smbus v4l-utils
```

把当前用户加入常用硬件访问组，然后重新登录终端：

```bash
sudo usermod -aG dialout,video,i2c $USER
```

如果系统没有 `i2c` 用户组，可以先跳过这一项，后续 I2C 检查命令使用 `sudo`。

## 拉取并安装项目

```bash
git clone https://github.com/bertholetnadine159-prog/Sunrise5-Based-Sea-Cucumber-Inspection-and-Suction-Harvest-Robot.git
cd Sunrise5-Based-Sea-Cucumber-Inspection-and-Suction-Harvest-Robot/sea_cucumber_robot

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

如果 RDK X5 已经有系统级 OpenCV、Horizon 或 D-Robotics 推理环境，也可以按系统镜像习惯使用系统 Python，但要保证 `numpy`、`opencv-python`、`PyYAML`、`pyserial`、`pymavlink`、`smbus2` 可导入。

## I2C 传感器检查

先确认系统暴露了哪些 I2C bus：

```bash
ls /dev/i2c-*
```

检查 VEML7700_1：

```bash
sudo i2cdetect -y 5
```

应看到地址 `0x10`。

检查 VEML7700_2：

```bash
sudo i2cdetect -y 0
```

应看到地址 `0x10`。

检查 MS5837-30BA：

```bash
sudo i2cdetect -y 1
```

应看到地址 `0x76` 或 `0x77`。如果没有看到：

- 先确认 MS5837 的 SDA/SCL 接到 `PIN33/PIN32`。
- 确认 RDK X5 的 `PIN32/PIN33` 已切到 I2C 复用功能，而不是 PWM/GPIO。
- 如果实际 bus 不是 `1`，把 `config/hardware.yaml` 中 `rdk_x5.i2c.ms5837_30ba.bus` 改为实测 bus。

## DS18B20 检查

确认 1-Wire 设备已经出现在 sysfs：

```bash
ls /sys/bus/w1/devices/28-*
cat /sys/bus/w1/devices/28-*/w1_slave
```

如果看不到 `28-*` 设备，需要先在 RDK X5 系统中启用 1-Wire，并确认上拉电阻、DATA 线和供电。DS18B20_2 当前按 `PIN13(GPIO27)` 作为 DATA，`PIN17` 保留为可能的供电脚，不在代码中硬判定真实接线。

## LO81MTW USB 超声检查

查看 USB 串口：

```bash
ls -l /dev/ttyUSB*
```

如果两个超声波传感器顺序会变，建议用 udev 规则固定名称，或在 `config/hardware.yaml` 中修改：

```yaml
ultrasonic_usb:
  front:
    port: /dev/ttyUSB0
  downward:
    port: /dev/ttyUSB1
```

运行项目传感器检查脚本：

```bash
python3 scripts/check_sensors.py
```

无硬件时可先验证软件接口：

```bash
python3 scripts/check_sensors.py --simulate
```

## 摄像头检查

先查看摄像头设备：

```bash
v4l2-ctl --list-devices
```

再运行项目检查脚本：

```bash
python3 scripts/check_cameras.py
```

如果前向摄像头和吸捕口摄像头顺序相反，修改 `config/hardware.yaml`：

```yaml
cameras:
  camera_1:
    device: 0
  camera_2:
    device: 1
```

## Pixhawk 检查

确认 Pixhawk 端口，常见为 `/dev/ttyACM0` 或 `/dev/ttyUSB0`：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

如果端口不是配置文件中的值，修改 `config/control.yaml` 的 Pixhawk 连接参数。然后发送一次中位 PWM 检查连接：

```bash
python3 scripts/check_pixhawk.py
```

仿真检查：

```bash
python3 scripts/check_pixhawk.py --simulate
```

## 运行仿真

在没有硬件或下水前，先跑完整状态机仿真：

```bash
bash scripts/run_simulation.sh
```

仿真会使用虚拟传感器、虚拟摄像头、虚拟 Pixhawk 和模拟 mask，适合检查状态机、PID、混控和吸捕流程。

## RDK X5 实机运行

确认传感器、摄像头、Pixhawk 都通过检查后启动实机任务：

```bash
bash scripts/run_robot.sh
```

程序入口等价于：

```bash
PYTHONPATH=src python -m sea_cucumber_robot.main --config-dir config --log-dir logs
```

日志默认写入 `logs/`。如果触发急停，状态机会进入 `EMERGENCY_STOP`，所有移动电机回中、吸捕电机停止、舵机回安全位置，不会自动恢复。

## 使用 RDK BIN 分割模型

RDK X5 上如果要使用仓库中的 `YOLO11_LBL.bin`，先确认系统环境能导入 Horizon 推理库：

```bash
python3 -c "from hobot_dnn import pyeasy_dnn; print('hobot_dnn ok')"
```

然后在 `config/vision.yaml` 中把视觉后端改为 `rdk_bin`，并确认模型路径指向仓库顶层的 `YOLO11_LBL.bin`。普通 PC 没有 `hobot_dnn` 时建议继续使用 `color_threshold` 或 `ultralytics` 后端调试。

## 常见修改位置

| 问题 | 修改文件 |
| --- | --- |
| I2C bus 不一致 | `config/hardware.yaml` |
| MS5837 32/33 没有 I2C | RDK X5 pinmux/设备树，随后同步 `config/hardware.yaml` |
| LO81MTW 串口顺序变化 | `config/hardware.yaml` |
| 摄像头 0/1 顺序相反 | `config/hardware.yaml` |
| Pixhawk 端口或波特率变化 | `config/control.yaml` |
| 分割模型后端或模型路径变化 | `config/vision.yaml` |
| 吸捕时长、状态机超时、日志策略 | `config/mission.yaml` |
