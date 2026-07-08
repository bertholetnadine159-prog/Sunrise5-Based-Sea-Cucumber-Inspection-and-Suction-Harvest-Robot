# Sunrise5-Based Sea Cucumber Inspection and Suction Harvest Robot

Team: Shenhai Electromagnetic Force

本仓库是基于 Sunrise5 / RDK X5 的海参检测与负压吸捕机器人硬件端程序。项目面向海参养殖、水下巡检和无损采捕场景，整合端侧 YOLO11 实例分割、Pixhawk 水下传感器数据读取、I2C 环境传感器采集、超声测距可视化和 RDK/Horizon BIN 模型部署能力。

机器人系统由三个开源部分组成：

- 本仓库：RDK X5 / Sunrise5 端侧检测、传感器读取和硬件调试程序。
- [Model-weight-conversion](https://github.com/bertholetnadine159-prog/Model-weight-conversion)：YOLO11 分割模型 ONNX 导出、RDK/Horizon BIN 转换和部署说明。
- [Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface](https://github.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface)：Flutter 机器人管理界面、Python 推理后端、WebSocket 视频回传和桌面/移动端控制页面。

## 系统定位

项目目标是为海参养殖与水下作业提供一套“检测 + 吸捕 + 环境感知 + 岸基管理”的完整机器人方案。RDK X5 负责端侧 AI 推理和多传感器数据采集，Pixhawk 负责姿态、运动与水下设备数据融合，地面端 SeaUI 负责视频监控、控制指令、日志和环境数据展示。

核心能力包括：

- YOLO11 海参实例分割：加载 `YOLO11_LBL.bin`，在 RDK/Horizon 环境中执行端侧分割推理。
- 水下传感器采集：读取 MS5837 压力/深度、VEML7700 光照、L08/L081MTW 超声测距等数据。
- Pixhawk 数据可视化：通过 MAVLink 读取 `RANGEFINDER`、`DISTANCE_SENSOR`、`SCALED_PRESSURE` 和自定义数值消息。
- I2C 硬件调试：支持 RDK X5 上多个 I2C 总线的自动扫描、读取和基础可视化。
- 与管理界面协同：可配合 SeaUI 实现岸基监控、控制操作、数据分析和设备管理。

## 程序文件

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

## 项目预览

### 模型转换与端侧部署

模型转换仓库提供 YOLO11 分割模型导出、BIN 转换和 RDK 端侧推理说明。本仓库中的 `YOLO11_LBL.bin` 可直接配合 RDK/Horizon 推理程序使用。

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Model-weight-conversion/main/f990ae1251f042863e950c4d86e5f24b.png" alt="模型转换示例图片" width="760">
</p>

### 机器人管理界面

SeaUI 管理界面提供 Windows 桌面端和移动端页面，覆盖登录、主控、控制操作、数据分析、管理员面板和系统设置等流程。

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.win/main_control/screen.png" alt="Windows 主控界面" width="900">
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.win/operate/screen.png" alt="Windows 控制操作界面" width="900">
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.win/data_analysis/screen.png" alt="Windows 数据分析界面" width="900">
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.app/main_control/screen.png" alt="移动端主控界面" width="360">
</p>

## 环境依赖

Python 依赖可先安装：

```bash
pip install -r requirements.txt
```

RDK / Horizon 端侧推理还需要目标设备已经安装并可导入：

- `hobot_dnn`
- 可选：`horizon_nn.debug`，用于量化敏感度分析

I2C 传感器程序建议在 RDK X5 / Linux 环境安装：

```bash
sudo apt update
sudo apt install -y i2c-tools python3-smbus
python3 -m pip install smbus2
```

MAVLink / 声呐可视化依赖：

```bash
python3 -m pip install pymavlink pyserial numpy matplotlib
```

## 快速开始

### 1. RDK YOLO11 BIN 分割推理

单张图片或图片目录推理：

```bash
python 2.py --model-path YOLO11_LBL.bin --test-img path/to/image_or_dir --img-save-path seg_results
```

使用验证集结构推理并统计指标：

```bash
python 2.py --model-path YOLO11_LBL.bin --data-dir valid --benchmark-bin
```

默认验证集结构：

```text
valid/
  images/
    xxx.jpg
  labels/
    xxx.txt
```

标签格式使用 YOLO segmentation 多边形格式：

```text
class_id x1 y1 x2 y2 x3 y3 ...
```

### 2. Pixhawk 水下数据读取与可视化

USB 直连 Pixhawk：

```bash
python3 pixhawk_direct_water_visualize.py --connection /dev/ttyACM0
```

TELEM 转 USB，适合无图形界面运行：

```bash
python3 pixhawk_direct_water_visualize.py --connection /dev/ttyUSB0 --baud 57600 --no-gui
```

程序会读取并记录压力、深度、测距、浊度或自定义 `NAMED_VALUE_FLOAT` 数据，可输出 CSV 和实时 PNG。

### 3. MS5837 压力/深度传感器

扫描 I2C 总线：

```bash
python3 rdk_x5_ms5837_test.py --scan
```

指定总线与地址读取：

```bash
python3 rdk_x5_ms5837_test.py --bus 5 --addr 0x76
```

轻量 `i2ctransfer` 版本：

```bash
python3 ms5837_read.py
```

### 4. VEML7700 光照传感器

```bash
python3 veml7700_read.py
```

### 5. 多 I2C 传感器联调

```bash
sudo python3 rdk_x5_i2c_sensor_gui.py
```

指定 MS5837 总线：

```bash
sudo python3 rdk_x5_i2c_sensor_gui.py --ms-bus 0 --ms-addr 0x76
```

### 6. L08 / L081MTW 超声测距

UART 模式：

```bash
python3 l08_test.py --port /dev/ttyUSB0 --protocol uart --raw
```

RS485 / Modbus 模式：

```bash
python3 l08_test.py --port /dev/ttyUSB0 --protocol modbus --address 1 --raw
```

## 常用参数

| 程序 | 常用参数 | 说明 |
| --- | --- | --- |
| `2.py` | `--model-path` | BIN 模型路径，默认可使用 `YOLO11_LBL.bin`。 |
| `2.py` | `--test-img` | 输入图片或图片目录。 |
| `2.py` | `--img-save-path` | 分割可视化结果保存目录。 |
| `2.py` | `--conf-thres` / `--iou-thres` / `--mask-thres` | 检测置信度、NMS IoU 和 mask 二值化阈值。 |
| `pixhawk_direct_water_visualize.py` | `--connection` | MAVLink 连接地址，例如 `/dev/ttyACM0`、`/dev/ttyUSB0`、`udp:127.0.0.1:14550`。 |
| `pixhawk_direct_water_visualize.py` | `--no-gui` | 不打开实时窗口，只保存 CSV/PNG。 |
| `rdk_x5_ms5837_test.py` | `--scan` | 自动扫描 MS5837 所在 I2C 总线。 |
| `l08_test.py` | `--protocol` | 选择 `uart` 或 `modbus`。 |

## 与其他仓库的协同

### 模型权重转换

如果需要从训练权重重新导出 ONNX 并转换为 RDK BIN，请参考：

- 仓库：[Model-weight-conversion](https://github.com/bertholetnadine159-prog/Model-weight-conversion)
- 主要内容：`onnx.py`、`Model deployment.py`、`YOLO11_LBL.bin`
- 典型流程：训练权重 `.pt` -> ONNX -> Horizon 量化 BIN -> RDK X5 端侧部署

### 管理界面

如果需要岸基控制端、视频回传和数据看板，请参考：

- 仓库：[Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface](https://github.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface)
- 技术栈：Flutter / Dart、Python、OpenCV、Ultralytics YOLO、ONNX Runtime、WebSocket
- 功能：登录、主控、控制操作、数据分析、管理员面板、系统设置

## 典型硬件连接

- RDK X5 / Sunrise5：运行本仓库 Python 程序，负责模型推理和传感器读取。
- Pixhawk：通过 USB、TELEM 或 UDP 发送 MAVLink 数据。
- MS5837：压力、温度和深度测量，常用 I2C 地址 `0x76` / `0x77`。
- VEML7700：环境光照测量，常用 I2C 地址 `0x10`。
- L08 / L081MTW：水下超声测距，可使用 UART 或 RS485/Modbus。

## 注意事项

- `2.py` 需要在 D-Robotics RDK/Horizon 环境运行，普通 PC 通常没有 `hobot_dnn`。
- 传感器脚本需要目标设备开放 I2C、串口或 MAVLink 连接权限，部分命令需要 `sudo`。
- 水下压力深度换算默认使用淡水密度 `997 kg/m^3`，海水环境可按实际密度修正。
- `YOLO11_LBL.bin` 已包含在仓库中，文件 SHA256 为 `BC66F9E8073D41A21863840F7A93F79A9B8D1A88AD5659C3A931F7D6E149D995`。

## 参考资料

- [Model-weight-conversion](https://github.com/bertholetnadine159-prog/Model-weight-conversion)
- [Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface](https://github.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface)
- SkyXZ：[万字长文，学弟一看就会的 RDKX5 模型转换及部署](https://www.cnblogs.com/SkyXZ/p/18681804)
