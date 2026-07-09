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
| Pixhawk 2.4.8 | `/dev/ttyACM0`，`115200` baud | 用 MAVLink 控制 MAIN OUT 1-8、AUX OUT 1-3 | `python3 scripts/check_pixhawk.py` 应能收到 heartbeat 并发送中位 PWM |
| 移动推进器 | MAIN OUT 1-8 | 8 路移动推进器，使用 `config/motors.yaml` 的 thruster mixer | 下水前必须校准每个推进器方向、反向和 PWM 范围 |
| 吸捕电机和舵机 | AUX1、AUX2、AUX3 | AUX1/AUX2 为吸捕仓电机，AUX3 为舵机 | 吸捕电机 50% 功率在 `SUCTION_CAPTURE` 状态输出 |

结论：按当前工程配置，VEML7700、DS18B20、LO81MTW、USB 摄像头和 Pixhawk 电机输出的软件映射是完整的；MS5837-30BA 已按 `PIN32/PIN33` 复用 I2C 方式配置，但它依赖 RDK X5 系统镜像的 pinmux/设备树设置。只要系统中对应 I2C bus 存在，并且 `i2cdetect` 能扫到 `0x76` 或 `0x77`，该接线方案就是可用的。

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

## 离线复制项目文件

如果 RDK X5 下载 GitHub 很慢，可以直接把本地文件复制到 RDK X5。推荐复制整个仓库目录，最省事，也不容易漏模型和脚本：

```text
Sunrise5-Based-Sea-Cucumber-Inspection-and-Suction-Harvest-Robot/
├── README.md
├── .gitignore
├── LICENSE
├── 2.py
├── YOLO11_LBL.bin
├── sea_cucumber_robot/
├── rdk_x5_i2c_sensor_gui.py
├── rdk_x5_ms5837_test.py
├── ms5837_read.py
├── veml7700_read.py
├── l08_test.py
└── pixhawk_direct_water_visualize.py
```

最小运行集合如下：

| 运行目标 | 必须复制 |
| --- | --- |
| 运行完整机器人程序 | `sea_cucumber_robot/` |
| 使用 RDK BIN 海参分割模型 | `sea_cucumber_robot/`、`YOLO11_LBL.bin`、`2.py` |
| 调试 I2C 传感器 | `sea_cucumber_robot/`、`rdk_x5_i2c_sensor_gui.py`、`rdk_x5_ms5837_test.py`、`ms5837_read.py`、`veml7700_read.py` |
| 调试 LO81MTW 超声波 | `sea_cucumber_robot/`、`l08_test.py` |
| 调试 Pixhawk 数据 | `sea_cucumber_robot/`、`pixhawk_direct_water_visualize.py` |
| 控制移动推进器、吸捕电机和舵机 | `sea_cucumber_robot/` |

电机控制相关文件都在 `sea_cucumber_robot/` 里，复制这个目录时会一起带走：

| 文件 | 作用 |
| --- | --- |
| `config/motors.yaml` | MAIN OUT 1-8、AUX OUT 1-3 输出映射，推进器位置、方向、PWM 范围和反向配置 |
| `config/control.yaml` | Pixhawk 串口、波特率、PID、搜索、接近、吸捕功率参数 |
| `src/sea_cucumber_robot/control/thruster_mixer.py` | 根据推进器位置和方向生成 6 自由度混控 PWM |
| `src/sea_cucumber_robot/control/pixhawk_mavlink.py` | 通过 MAVLink `MAV_CMD_DO_SET_SERVO` 输出 PWM |
| `src/sea_cucumber_robot/control/suction_controller.py` | 控制 AUX1/AUX2 吸捕电机和 AUX3 舵机 |
| `scripts/check_pixhawk.py` | 检查 Pixhawk heartbeat，并向 MAIN OUT 推进器发送中位 PWM |

不要复制这些运行产物：

```text
.venv/
__pycache__/
logs/*.log
seg_results/
outputs/
*.mp4
*.avi
```

如果从 U 盘复制到 RDK X5 桌面，可以这样放置：

```bash
mkdir -p ~/桌面/sea_robot
cp -r /media/$USER/<U盘名称>/Sunrise5-Based-Sea-Cucumber-Inspection-and-Suction-Harvest-Robot/* ~/桌面/sea_robot/
cd ~/桌面/sea_robot/sea_cucumber_robot
```

随后安装依赖并运行：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 scripts/check_sensors.py --simulate
```

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

如果 `sudo i2cdetect -y 1` 输出中没有 `76` 或 `77`，但系统存在 `/dev/i2c-0` 到 `/dev/i2c-8`，先扫描所有 bus：

```bash
for dev in /dev/i2c-*; do
  bus=${dev#/dev/i2c-}
  echo "===== i2c-$bus ====="
  sudo i2cdetect -y -r "$bus"
done
```

判读方式：

- 看到 `76` 或 `77`：MS5837 已响应，把 `config/hardware.yaml` 中 `rdk_x5.i2c.ms5837_30ba.bus` 改成对应 bus。
- 任何 bus 都看不到 `76/77`：优先检查 `PIN32/PIN33` 的 pinmux、传感器 3.3 V 供电、GND 共地、SDA/SCL 是否接反、线缆和水密接头。
- `Warning: Can't use SMBus Quick Write command` 不是 MS5837 未识别的原因，只表示当前适配器不支持 SMBus quick write，使用 `-r` 扫描即可减少这个提示。
- 表格里 `--` 表示该地址没有设备响应，`UU` 表示该地址已被内核驱动占用。

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

## Pixhawk 和电机控制检查

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

电机输出映射在 `config/motors.yaml`：

| Pixhawk 输出 | MAVLink servo channel | 当前用途 |
| --- | --- | --- |
| MAIN OUT 1-8 | `1` 到 `8` | 8 路移动推进器 |
| AUX OUT 1 | `9` | 吸捕仓电机 1 |
| AUX OUT 2 | `10` | 吸捕仓电机 2 |
| AUX OUT 3 | `11` | 舵机 |

第一次实机电机检查建议按这个顺序：

1. 先断开推进器和吸捕电机动力电源，或确保机器人固定且推进器不会造成危险。
2. 运行 `python3 scripts/check_pixhawk.py --simulate`，确认软件路径无误。
3. 连接 Pixhawk，但保持电机安全，运行 `python3 scripts/check_pixhawk.py`，该脚本只向 MAIN OUT 1-8 发送中位 PWM。
4. 核对 `config/motors.yaml` 中每个 `pixhawk_output` 是否和真实接线一致。
5. 核对每个推进器的 `position`、`direction`、`reversed`、`neutral_pwm`、`min_pwm`、`max_pwm` 后，再允许自动任务控制电机。

AUX1/AUX2 吸捕电机由 `src/sea_cucumber_robot/control/suction_controller.py` 控制，任务进入 `SUCTION_CAPTURE` 后按 `config/control.yaml` 中的 `suction.power_percent: 50` 输出 50% 功率。当前 AUX1/AUX2 配置为 `min_pwm: 1000`、`max_pwm: 2000`，所以 50% 对应约 `1500 us`。

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
