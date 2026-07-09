# Sunrise5-Based Sea Cucumber Inspection and Suction Harvest Robot

Team: Shenhai Electromagnetic Force

Language: [English](README.md) | [中文](README_cn.md)

## 项目相关开源仓库

本项目由多个开源仓库共同组成，以下两个链接也是本项目的重要组成部分：

- [Model-weight-conversion](https://github.com/bertholetnadine159-prog/Model-weight-conversion)：海参 YOLO11 分割模型的 ONNX 导出、RDK/Horizon BIN 转换和端侧部署。
- [Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface](https://github.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface)：海参检测及吸捕机器人管理界面、视频回传、数据看板和上位机控制系统。

本仓库面向“水下海参检测及吸捕机器人”项目，包含两类内容：

- `sea_cucumber_robot/`：新整理的完整工程化代码项目，包含配置、传感器、视觉分割、Pixhawk 控制、推进器混控、吸捕流程、状态机、脚本和测试。
- 仓库根目录下的硬件调试脚本：用于 RDK X5 / Sunrise5 上的模型推理、I2C 传感器读取、Pixhawk MAVLink 数据可视化和单项硬件验证。

系统围绕“端侧 AI 检测、视觉对中、超声测距接近、近距离吸捕、环境数据记录、Pixhawk 输出控制”构建，可配合上位机管理界面和模型转换仓库组成完整开源方案。

## 传感器接口文档

传感器硬件连接、通信协议、配置节点、驱动文件和输出字段已开放在文档中：

[sea_cucumber_robot/docs/sensor_interfaces.md](sea_cucumber_robot/docs/sensor_interfaces.md)

已公开接口包括 MS5837-30BA、两路 LO81MTW 水下超声、两路 VEML7700、两路 DS18B20 和两路 USB 摄像头。

RDK X5 接线核对表和实机运行步骤：

[sea_cucumber_robot/docs/rdk_x5_wiring_and_run.md](sea_cucumber_robot/docs/rdk_x5_wiring_and_run.md)

## 完整工程入口

建议优先使用完整工程目录：

```text
sea_cucumber_robot/
├── config/                 # 硬件、输出、视觉、控制和任务参数
├── docs/                   # 传感器接口等开源文档
├── src/sea_cucumber_robot/ # Python 机器人软件包
├── scripts/                # 运行与硬件检查脚本
├── tests/                  # 核心逻辑单元测试
└── logs/                   # 运行日志目录
```

完整工程 README：

[sea_cucumber_robot/README.md](sea_cucumber_robot/README.md)

完整工程实现的核心能力：

- 读取 MS5837-30BA、两路 LO81MTW 水下超声、两路 VEML7700、两路 DS18B20、两路 USB 摄像头。
- 使用摄像头 1 前向检测海参，计算分割 mask 中心并与画面中心线对齐。
- 使用前向超声控制机器人接近到 `5.5 cm`。
- 到达近距离后关闭摄像头 1，打开摄像头 2，继续用吸捕口近距离摄像头对中。
- 通过 Pixhawk 2.4.8 控制 MAIN OUT 1-8 移动推进器、AUX OUT 1-2 吸捕电机、AUX OUT 3 舵机。
- 使用配置化 thruster mixer，不在业务逻辑中写死推进器方向。
- 严格实现 `INIT`、`SEARCH_WITH_CAMERA_1`、`ALIGN_WITH_CAMERA_1`、`APPROACH_TO_5_5CM`、`SWITCH_TO_CAMERA_2`、`ALIGN_WITH_CAMERA_2`、`SUCTION_CAPTURE`、`COMPLETE`、`EMERGENCY_STOP` 状态机。

## 快速运行

安装完整工程依赖：

```bash
cd sea_cucumber_robot
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

无硬件仿真运行：

```bash
bash scripts/run_simulation.sh
```

实机前逐项检查：

```bash
python3 scripts/check_sensors.py
python3 scripts/check_cameras.py
python3 scripts/check_pixhawk.py
```

实机运行：

```bash
bash scripts/run_robot.sh
```

测试：

```bash
PYTHONPATH=src python -m unittest discover -s tests
```

## 配置文件

完整工程所有硬件连接都在配置文件中维护，避免在业务逻辑中硬编码端口和引脚：

- `sea_cucumber_robot/config/hardware.yaml`：RDK_X5 传感器接线、I2C bus、USB 超声、摄像头索引。
- `sea_cucumber_robot/config/motors.yaml`：Pixhawk MAIN/AUX 输出映射、推进器位置、方向、PWM 范围、反向配置。
- `sea_cucumber_robot/config/vision.yaml`：分割模型后端、模型路径、阈值和调试输出。
- `sea_cucumber_robot/config/control.yaml`：Pixhawk、PID、搜索、接近和吸捕参数。
- `sea_cucumber_robot/config/mission.yaml`：状态机循环频率、吸捕时间、超时、急停和日志策略。

DS18B20_2 的接线说明存在歧义，配置中保留 `power_pin: 17`、`data_pin: 13`、`gpio: 27`，代码默认使用 GPIO27 作为 DATA，但实机前必须人工确认。

## 根目录硬件调试脚本

| 文件 | 说明 |
| --- | --- |
| `2.py` | RDK/Horizon YOLO11 分割 BIN 模型推理程序，支持图片/目录输入、mask 可视化、评估和测速。 |
| `YOLO11_LBL.bin` | 已转换的 YOLO11 海参分割量化 BIN 模型，供 RDK X5 端侧推理使用。 |
| `pixhawk_direct_water_visualize.py` | 通过 MAVLink 读取 Pixhawk 发布的水下传感器数据，并保存 CSV/PNG 或实时可视化。 |
| `rdk_x5_i2c_sensor_gui.py` | 使用 `i2c-tools` 同时读取 VEML7700 和 MS5837，适合 RDK X5 硬件联调。 |
| `rdk_x5_ms5837_test.py` | 基于 `smbus2` 的 MS5837-02BA / MS5837-30BA 压力、温度、深度读取测试。 |
| `ms5837_read.py` | 使用 `i2ctransfer` 直接读取 MS5837 的轻量测试程序。 |
| `veml7700_read.py` | 使用 `i2cget` / `i2cset` 读取 VEML7700 光照和白光数据。 |
| `l08_test.py` | L08 / L081MTW 水下超声测距测试，支持 UART、RS485/Modbus、CSV 记录和可视化。 |

## 模型与界面协同

上面的两个项目仓库提供模型转换和上位机界面能力：

- [Model-weight-conversion](https://github.com/bertholetnadine159-prog/Model-weight-conversion)：YOLO11 分割模型 ONNX 导出、RDK/Horizon BIN 转换和部署说明。
- [Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface](https://github.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface)：Flutter 机器人管理界面、Python 推理后端、WebSocket 视频回传和桌面/移动端控制页面。

模型转换示例：

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Model-weight-conversion/main/f990ae1251f042863e950c4d86e5f24b.png" alt="模型转换示例图片" width="760">
</p>

SeaUI 主控界面：

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.win/main_control/screen.png" alt="Windows 主控界面" width="900">
</p>

SeaUI 控制操作界面：

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.win/operate/screen.png" alt="Windows 控制操作界面" width="900">
</p>

SeaUI 数据分析界面：

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.win/data_analysis/screen.png" alt="Windows 数据分析界面" width="900">
</p>

移动端主控界面：

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.app/main_control/screen.png" alt="移动端主控界面" width="360">
</p>

## 注意事项

- `2.py` 和完整工程的 `rdk_bin` 视觉后端需要在 D-Robotics RDK/Horizon 环境中运行，普通 PC 通常没有 `hobot_dnn`。
- 传感器脚本需要目标设备开放 I2C、串口或 MAVLink 连接权限，部分命令需要 `sudo`。
- `config/motors.yaml` 中的推进器位置和方向向量必须根据真实电机布局校准后再下水。
- `YOLO11_LBL.bin` 已包含在仓库中，文件 SHA256 为 `BC66F9E8073D41A21863840F7A93F79A9B8D1A88AD5659C3A931F7D6E149D995`。

## 参考资料

- [Model-weight-conversion](https://github.com/bertholetnadine159-prog/Model-weight-conversion)
- [Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface](https://github.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface)
- SkyXZ：[万字长文，学弟一看就会的 RDKX5 模型转换及部署](https://www.cnblogs.com/SkyXZ/p/18681804)
