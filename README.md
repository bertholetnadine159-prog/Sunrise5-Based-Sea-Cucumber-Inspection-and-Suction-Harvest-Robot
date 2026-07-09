# Sunrise5-Based Sea Cucumber Inspection and Suction Harvest Robot

Team: Shenhai Electromagnetic Force

Language: [English](README.md) | [中文](README_cn.md)

## Related Open-Source Repositories

This project is composed of multiple open-source repositories. The following two repositories are also important parts of this project:

- [Model-weight-conversion](https://github.com/bertholetnadine159-prog/Model-weight-conversion): ONNX export, RDK/Horizon BIN conversion, and edge deployment for the sea cucumber YOLO11 segmentation model.
- [Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface](https://github.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface): management interface, video streaming, data dashboard, and host-side control system for the sea cucumber inspection and suction harvest robot.

This repository targets an underwater sea cucumber inspection and suction harvest robot. It contains two types of content:

- `sea_cucumber_robot/`: a complete engineering project with configuration, sensors, vision segmentation, Pixhawk control, thruster mixing, suction workflow, mission state machine, scripts, and tests.
- Hardware debugging scripts in the repository root: model inference, I2C sensor reading, Pixhawk MAVLink data visualization, and single-hardware validation for RDK X5 / Sunrise5.

The system is built around edge AI detection, visual alignment, ultrasonic distance-based approach, close-range suction capture, environmental data logging, and Pixhawk output control. It can work together with the management interface repository and the model conversion repository as a complete open-source solution.

## Sensor Interface Documentation

Sensor hardware wiring, communication protocols, configuration nodes, driver files, and runtime output fields are documented here:

[sea_cucumber_robot/docs/sensor_interfaces.md](sea_cucumber_robot/docs/sensor_interfaces.md)

The documented interfaces include MS5837-30BA, two LO81MTW underwater ultrasonic sensors, two VEML7700 light sensors, two DS18B20 temperature sensors, and two USB cameras.

RDK X5 wiring checklist and real-device running guide:

[sea_cucumber_robot/docs/rdk_x5_wiring_and_run.md](sea_cucumber_robot/docs/rdk_x5_wiring_and_run.md)

## Complete Project Entry

It is recommended to use the complete project directory first:

```text
sea_cucumber_robot/
├── config/                 # Hardware, outputs, vision, control, and mission parameters
├── docs/                   # Open documentation such as sensor interfaces
├── src/sea_cucumber_robot/ # Python robot software package
├── scripts/                # Running and hardware-check scripts
├── tests/                  # Unit tests for core logic
└── logs/                   # Runtime log directory
```

Complete project README:

[sea_cucumber_robot/README.md](sea_cucumber_robot/README.md)

Core capabilities implemented in the complete project:

- Read MS5837-30BA, two LO81MTW underwater ultrasonic sensors, two VEML7700 light sensors, two DS18B20 temperature sensors, and two USB cameras.
- Use camera 1 for forward sea cucumber detection, calculate the segmentation mask center, and align it with the image center line.
- Use the forward ultrasonic sensor to approach the target to `5.5 cm`.
- After reaching close range, close camera 1, open camera 2, and continue alignment with the suction-mouth close-range camera.
- Control Pixhawk 2.4.8 MAIN OUT 1-8 mobile thrusters, AUX OUT 1-2 suction motors, and AUX OUT 3 servo.
- Use a configurable thruster mixer instead of hard-coding thruster directions in business logic.
- Strictly implement the `INIT`, `SEARCH_WITH_CAMERA_1`, `ALIGN_WITH_CAMERA_1`, `APPROACH_TO_5_5CM`, `SWITCH_TO_CAMERA_2`, `ALIGN_WITH_CAMERA_2`, `SUCTION_CAPTURE`, `COMPLETE`, and `EMERGENCY_STOP` mission states.

## Quick Start

Install dependencies for the complete project:

```bash
cd sea_cucumber_robot
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Run hardware-free simulation:

```bash
bash scripts/run_simulation.sh
```

Check hardware step by step before real-device operation:

```bash
python3 scripts/check_sensors.py
python3 scripts/check_cameras.py
python3 scripts/check_pixhawk.py
```

Run on the real robot:

```bash
bash scripts/run_robot.sh
```

Run tests:

```bash
PYTHONPATH=src python -m unittest discover -s tests
```

## Configuration Files

All hardware connections in the complete project are maintained in configuration files, avoiding hard-coded ports and pins in business logic:

- `sea_cucumber_robot/config/hardware.yaml`: RDK_X5 sensor wiring, I2C buses, USB ultrasonic sensors, and camera indices.
- `sea_cucumber_robot/config/motors.yaml`: Pixhawk MAIN/AUX output mapping, thruster positions, directions, PWM ranges, and reverse settings.
- `sea_cucumber_robot/config/vision.yaml`: segmentation backend, model path, thresholds, and debug output.
- `sea_cucumber_robot/config/control.yaml`: Pixhawk, PID, search, approach, and suction parameters.
- `sea_cucumber_robot/config/mission.yaml`: state-machine loop rate, suction duration, timeouts, emergency-stop settings, and logging policy.

The wiring note for DS18B20_2 is ambiguous. The configuration keeps `power_pin: 17`, `data_pin: 13`, and `gpio: 27`. The code uses GPIO27 as DATA by default, but the real wiring must be manually confirmed before real-device testing.

## Root Hardware Debugging Scripts

| File | Description |
| --- | --- |
| `2.py` | RDK/Horizon YOLO11 segmentation BIN model inference program, supporting image/directory input, mask visualization, evaluation, and speed testing. |
| `YOLO11_LBL.bin` | Converted YOLO11 sea cucumber segmentation quantized BIN model for RDK X5 edge inference. |
| `pixhawk_direct_water_visualize.py` | Reads underwater sensor data published through Pixhawk via MAVLink and saves CSV/PNG files or visualizes data in real time. |
| `rdk_x5_i2c_sensor_gui.py` | Reads VEML7700 and MS5837 with `i2c-tools`, suitable for RDK X5 hardware joint debugging. |
| `rdk_x5_ms5837_test.py` | `smbus2`-based pressure, temperature, and depth reading test for MS5837-02BA / MS5837-30BA. |
| `ms5837_read.py` | Lightweight MS5837 test program using `i2ctransfer`. |
| `veml7700_read.py` | Reads VEML7700 light and white-light data with `i2cget` / `i2cset`. |
| `l08_test.py` | L08 / L081MTW underwater ultrasonic ranging test with UART, RS485/Modbus, CSV logging, and visualization support. |

## Model and Interface Collaboration

The two related repositories provide model conversion and host-side interface capabilities:

- [Model-weight-conversion](https://github.com/bertholetnadine159-prog/Model-weight-conversion): YOLO11 segmentation model ONNX export, RDK/Horizon BIN conversion, and deployment instructions.
- [Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface](https://github.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface): Flutter robot management interface, Python inference backend, WebSocket video streaming, and desktop/mobile control pages.

Model conversion example:

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Model-weight-conversion/main/f990ae1251f042863e950c4d86e5f24b.png" alt="Model conversion example" width="760">
</p>

SeaUI main-control interface:

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.win/main_control/screen.png" alt="Windows main-control interface" width="900">
</p>

SeaUI operation interface:

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.win/operate/screen.png" alt="Windows operation interface" width="900">
</p>

SeaUI data-analysis interface:

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.win/data_analysis/screen.png" alt="Windows data-analysis interface" width="900">
</p>

Mobile main-control interface:

<p align="center">
  <img src="https://raw.githubusercontent.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface/main/1.app/main_control/screen.png" alt="Mobile main-control interface" width="360">
</p>

## Notes

- `2.py` and the complete project's `rdk_bin` vision backend must run in a D-Robotics RDK/Horizon environment. A normal PC usually does not provide `hobot_dnn`.
- Sensor scripts require I2C, serial, or MAVLink access permissions on the target device. Some commands may require `sudo`.
- Thruster position and direction vectors in `config/motors.yaml` must be calibrated according to the real motor layout before underwater testing.
- `YOLO11_LBL.bin` is included in this repository. Its SHA256 is `BC66F9E8073D41A21863840F7A93F79A9B8D1A88AD5659C3A931F7D6E149D995`.

## References

- [Model-weight-conversion](https://github.com/bertholetnadine159-prog/Model-weight-conversion)
- [Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface](https://github.com/bertholetnadine159-prog/Sea-Cucumber-Inspection-Harvest-Robot-Management-Interface)
- SkyXZ: [A long-form guide to RDK X5 model conversion and deployment](https://www.cnblogs.com/SkyXZ/p/18681804)
