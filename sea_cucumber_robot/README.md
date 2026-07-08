# 水下海参检测及吸捕机器人完整代码项目

本项目面向 RDK_X5 + Pixhawk 2.4.8 的水下海参检测及负压吸捕机器人，包含传感器读取、摄像头管理、海参分割检测、mask 对中、推进器混控、Pixhawk MAVLink 输出、吸捕电机控制和自动任务状态机。

## 目录结构

```text
sea_cucumber_robot/
├── config/                 # 所有硬件、输出、视觉、控制和任务参数
├── src/sea_cucumber_robot/ # 可运行 Python 包
├── scripts/                # 上机运行和硬件检查脚本
├── tests/                  # 核心逻辑单元测试
└── logs/                   # 运行日志
```

## 硬件配置

所有接线、端口、I2C bus、摄像头索引和 Pixhawk 输出都写在配置文件中：

- `config/hardware.yaml`：RDK_X5 传感器、USB 超声、USB 摄像头
- `config/motors.yaml`：MAIN OUT 1-8、AUX OUT 1-3 和推进器混控参数
- `config/vision.yaml`：分割模型后端、模型路径、阈值和调试输出
- `config/control.yaml`：Pixhawk、PID、搜索、接近和吸捕参数
- `config/mission.yaml`：任务状态机、超时、急停和日志参数

DS18B20_2 的接线说明存在歧义，配置文件中保留了：

- `power_pin: 17`
- `data_pin: 13`
- `gpio: 27`

代码默认把 GPIO27 作为 DS18B20_2 的 DATA 线使用，但 README 和配置均明确要求实机前人工确认，不在业务逻辑中擅自确定真实接线。

## 自动任务状态机

状态机在 `src/sea_cucumber_robot/mission/mission_state_machine.py` 中实现，状态严格对应任务流程：

1. `INIT`
2. `SEARCH_WITH_CAMERA_1`
3. `ALIGN_WITH_CAMERA_1`
4. `APPROACH_TO_5_5CM`
5. `SWITCH_TO_CAMERA_2`
6. `ALIGN_WITH_CAMERA_2`
7. `SUCTION_CAPTURE`
8. `COMPLETE`
9. `EMERGENCY_STOP`

`EMERGENCY_STOP` 会立即使 MAIN OUT 推进器回中、AUX 吸捕电机停止、舵机回安全位置，并写入急停标志文件。急停后不自动恢复。

## 安装

```bash
cd sea_cucumber_robot
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

RDK/Horizon 端如果使用 `backend: rdk_bin`，还需要系统已提供 `hobot_dnn`。

## 仿真运行

没有硬件时可以先跑仿真状态机：

```bash
cd sea_cucumber_robot
bash scripts/run_simulation.sh
```

仿真模式会使用虚拟传感器、虚拟摄像头画面、虚拟 Pixhawk 输出和模拟 mask，用来验证状态机、PID、混控和吸捕流程。

## 实机运行

先逐项检查硬件：

```bash
python3 scripts/check_sensors.py
python3 scripts/check_cameras.py
python3 scripts/check_pixhawk.py
```

确认配置无误后运行：

```bash
bash scripts/run_robot.sh
```

## 分割模型后端

`config/vision.yaml` 支持三种后端：

- `color_threshold`：台架调试用颜色阈值分割，不依赖模型。
- `ultralytics`：用于 `.pt` 或 `.onnx` 模型的普通 PC 调试。
- `rdk_bin`：在 RDK/Horizon 上加载 `YOLO11_LBL.bin`，并复用仓库顶层 `2.py` 中已有的 YOLO11 分割后处理逻辑。

## LO81MTW 水下超声波

`src/sea_cucumber_robot/sensors/ultrasonic_usb.py` 同时实现了官方 Arduino 示例中的两类协议：

- FF UART：`FF Data_H Data_L SUM`
- Modbus：`01 03 01 01 00 01 D4 36` 读取实时距离

前向超声用于接近到 `0.055 m`，下向超声用于离底距离或下潜高度记录。切换到摄像头 2 后，前向超声可能低于量程并返回错误值，状态机不会因此立刻退出任务。

## Pixhawk 输出

Pixhawk 输出使用 MAVLink `MAV_CMD_DO_SET_SERVO`：

- MAIN OUT 1-8：移动推进器
- AUX OUT 1：吸捕仓电机 1
- AUX OUT 2：吸捕仓电机 2
- AUX OUT 3：舵机

推进器不在代码里写死方向，而是从 `config/motors.yaml` 的 `position` 和 `direction` 构建 6 自由度混控矩阵。下水前必须按真实电机布局修正这些向量。

## 测试

```bash
cd sea_cucumber_robot
PYTHONPATH=src python -m unittest discover -s tests
```

测试覆盖 PID、mask 几何、推进器混控和仿真状态机闭环，不依赖额外测试框架。
