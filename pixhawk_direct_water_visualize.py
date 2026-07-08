#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RDK X5 直接通过 MAVLink 读取 Pixhawk 已发布的水下传感器数据并可视化

不需要 Pixhawk Lua，前提是 Pixhawk 已经把数据发布成 MAVLink 消息。

支持读取：
1. RANGEFINDER
   - distance: m
   - voltage: V，如果可用
   - 可用于把 ADC6.6 上的浊度 AO 伪装成 Analog RangeFinder 后读取

2. DISTANCE_SENSOR
   - current_distance: cm

3. SCALED_PRESSURE / SCALED_PRESSURE2 / SCALED_PRESSURE3
   - press_abs: hPa / mbar
   - temperature: cdegC
   - 程序会用启动时的压力做水面基准，估算深度

4. NAMED_VALUE_FLOAT
   - 兼容未来自定义数据，例如 TURB_V、TURB_NTU、DEPTH_M

运行示例：
    python3 pixhawk_direct_water_visualize.py --connection /dev/ttyACM0

SSH / 无图形界面：
    python3 pixhawk_direct_water_visualize.py --connection /dev/ttyACM0 --no-gui

TELEM 转 USB：
    python3 pixhawk_direct_water_visualize.py --connection /dev/ttyUSB0 --baud 57600 --no-gui
"""

import os
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")

import argparse
import csv
import math
import signal
import sys
import time
from collections import deque
from datetime import datetime

try:
    from pymavlink import mavutil
except ImportError:
    print("错误：没有安装 pymavlink，请先运行：pip3 install pymavlink")
    sys.exit(1)


running = True


MESSAGE_IDS = {
    "SCALED_PRESSURE": 29,
    "SCALED_PRESSURE2": 137,
    "SCALED_PRESSURE3": 143,
    "DISTANCE_SENSOR": 132,
    "RANGEFINDER": 173,
    "NAMED_VALUE_FLOAT": 251,
}


def handle_signal(signum, frame):
    global running
    running = False


def parse_args():
    parser = argparse.ArgumentParser(
        description="RDK X5 direct MAVLink visualizer for Pixhawk water sensors"
    )

    parser.add_argument(
        "--connection",
        type=str,
        default="/dev/ttyACM0",
        help="MAVLink 连接，例如 /dev/ttyACM0、/dev/ttyUSB0、udp:127.0.0.1:14550"
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="串口波特率，USB 通常无所谓；TELEM 常见 57600 或 115200"
    )

    parser.add_argument(
        "--hz",
        type=float,
        default=5.0,
        help="请求 Pixhawk 发送相关 MAVLink 消息的频率，默认 5Hz"
    )

    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="RDK 主循环刷新间隔，默认 0.1 秒"
    )

    parser.add_argument(
        "--window",
        type=float,
        default=120.0,
        help="图像显示最近多少秒数据，默认 120 秒"
    )

    parser.add_argument(
        "--csv",
        type=str,
        default="pixhawk_direct_water_log.csv",
        help="CSV 日志文件名"
    )

    parser.add_argument(
        "--png",
        type=str,
        default="pixhawk_direct_water_latest.png",
        help="PNG 图像文件名"
    )

    parser.add_argument(
        "--png-every",
        type=float,
        default=5.0,
        help="无 GUI 模式下每隔多少秒保存一次 PNG"
    )

    parser.add_argument(
        "--print-every",
        type=float,
        default=1.0,
        help="每隔多少秒打印一次数据"
    )

    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="不弹出实时窗口，只保存 CSV 和 PNG"
    )

    parser.add_argument(
        "--fresh-water",
        action="store_true",
        help="使用淡水密度 997 kg/m^3，默认启用"
    )

    parser.add_argument(
        "--density",
        type=float,
        default=997.0,
        help="水密度 kg/m^3，淡水约 997，海水约 1029，默认 997"
    )

    parser.add_argument(
        "--surface-pressure",
        type=float,
        default=None,
        help="水面压力 mbar。不填则启动后自动用前 N 个压力样本校准"
    )

    parser.add_argument(
        "--surface-samples",
        type=int,
        default=50,
        help="自动校准水面压力的样本数，默认 50"
    )

    parser.add_argument(
        "--pressure-message",
        choices=["any", "SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3"],
        default="any",
        help="指定用哪个压力消息估算深度，默认 any"
    )

    parser.add_argument(
        "--range-distance-as-voltage",
        action="store_true",
        default=True,
        help="如果 RANGEFINDER.voltage 为 0，则把 RANGEFINDER.distance 当作电压。默认启用"
    )

    return parser.parse_args()


def fmt(value, digits=3):
    if value is None:
        return "None"
    if isinstance(value, float) and math.isnan(value):
        return "None"
    return f"{value:.{digits}f}"


def finite_or_none(value):
    if value is None:
        return None
    try:
        value = float(value)
    except Exception:
        return None
    if math.isnan(value) or math.isinf(value):
        return None
    return value


def value_or_nan(value):
    value = finite_or_none(value)
    if value is None:
        return float("nan")
    return value


def turbidity_voltage_to_ntu(voltage):
    """
    常见浊度模块参考公式：
    NTU = -1120.4 * V^2 + 5742.3 * V - 4352.9

    这只是估算趋势，不是标定后的精确 NTU。
    """
    voltage = finite_or_none(voltage)
    if voltage is None:
        return None

    ntu = -1120.4 * voltage * voltage + 5742.3 * voltage - 4352.9

    if ntu < 0:
        ntu = 0.0

    if ntu > 3000:
        ntu = 3000.0

    return ntu


def pressure_to_depth_m(pressure_mbar, surface_pressure_mbar, density):
    if pressure_mbar is None or surface_pressure_mbar is None:
        return None

    delta_pa = (pressure_mbar - surface_pressure_mbar) * 100.0
    depth = delta_pa / (density * 9.80665)

    if depth < 0:
        depth = 0.0

    return depth


def decode_named_value_name(raw_name):
    if isinstance(raw_name, bytes):
        return raw_name.decode("ascii", errors="ignore").strip("\x00 ").strip()
    return str(raw_name).strip("\x00 ").strip()


def connect_mavlink(connection, baud):
    print("=" * 70)
    print("正在连接 Pixhawk MAVLink")
    print(f"connection: {connection}")
    print(f"baud: {baud}")
    print("=" * 70)

    master = mavutil.mavlink_connection(
        connection,
        baud=baud,
        autoreconnect=True
    )

    print("等待 heartbeat ...")
    heartbeat = master.wait_heartbeat(timeout=15)

    if heartbeat is None:
        raise RuntimeError("15 秒内没有收到 heartbeat，请检查 Pixhawk 连接、串口权限和波特率")

    print(
        f"已连接 Pixhawk：system={master.target_system}, "
        f"component={master.target_component}"
    )

    return master


def request_message_interval(master, message_name, hz):
    if hz <= 0:
        return

    msg_id = MESSAGE_IDS.get(message_name)
    if msg_id is None:
        return

    interval_us = int(1_000_000 / hz)

    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0
        )
        print(f"已请求 {message_name} 以 {hz:.1f}Hz 发送")
    except Exception as e:
        print(f"请求 {message_name} 发送频率失败：{e}")


def request_all_message_intervals(master, hz):
    for name in [
        "RANGEFINDER",
        "DISTANCE_SENSOR",
        "SCALED_PRESSURE",
        "SCALED_PRESSURE2",
        "SCALED_PRESSURE3",
        "NAMED_VALUE_FLOAT",
    ]:
        request_message_interval(master, name, hz)
        time.sleep(0.05)


def prepare_csv(csv_path):
    file_exists = os.path.exists(csv_path)
    file_has_content = file_exists and os.path.getsize(csv_path) > 0

    csv_file = open(csv_path, "a", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)

    if not file_has_content:
        writer.writerow([
            "timestamp",
            "elapsed_s",
            "turbidity_voltage_v",
            "turbidity_ntu_est",
            "rangefinder_distance_m",
            "distance_sensor_m",
            "pressure_mbar",
            "surface_pressure_mbar",
            "depth_from_pressure_m",
            "temperature_c",
            "last_message"
        ])
        csv_file.flush()

    return csv_file, writer


def setup_plot(headless):
    if headless:
        import matplotlib
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 9))

    ax_v, ax_ntu, ax_depth, ax_env = axes

    lines = {}

    lines["turb_v"], = ax_v.plot([], [], label="Turbidity voltage / V")
    ax_v.set_title("Pixhawk Direct MAVLink Water Sensors")
    ax_v.set_ylabel("Voltage / V")
    ax_v.grid(True)
    ax_v.legend(loc="upper right")

    lines["turb_ntu"], = ax_ntu.plot([], [], label="Turbidity NTU estimate")
    ax_ntu.set_ylabel("NTU estimate")
    ax_ntu.grid(True)
    ax_ntu.legend(loc="upper right")

    lines["depth_m"], = ax_depth.plot([], [], label="Pressure depth / m")
    lines["distance_sensor_m"], = ax_depth.plot([], [], label="Distance sensor / m")
    lines["rangefinder_distance_m"], = ax_depth.plot([], [], label="Rangefinder distance / m")
    ax_depth.set_ylabel("Depth / Distance")
    ax_depth.grid(True)
    ax_depth.legend(loc="upper right")

    lines["pressure_mbar"], = ax_env.plot([], [], label="Pressure / mbar")
    lines["temp_c"], = ax_env.plot([], [], label="Temperature / C")
    ax_env.set_xlabel("Time / s")
    ax_env.set_ylabel("Pressure / Temp")
    ax_env.grid(True)
    ax_env.legend(loc="upper right")

    fig.tight_layout()

    if not headless:
        plt.ion()
        plt.show(block=False)

    return plt, fig, axes, lines


def update_plot(fig, axes, lines, xs, data, window_s):
    x_list = list(xs)

    for key, line in lines.items():
        line.set_data(x_list, list(data[key]))

    if len(x_list) >= 2:
        x_max = max(x_list[-1], 1.0)
        x_min = max(0.0, x_max - window_s)
    else:
        x_min = 0.0
        x_max = 1.0

    for ax in axes:
        ax.set_xlim(x_min, x_max)
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)

    fig.canvas.draw_idle()


def process_message(msg, state, args):
    msg_type = msg.get_type()

    if msg_type == "BAD_DATA":
        return

    state["last_message"] = msg_type

    if msg_type == "RANGEFINDER":
        distance = finite_or_none(getattr(msg, "distance", None))
        voltage = finite_or_none(getattr(msg, "voltage", None))

        state["rangefinder_distance_m"] = distance

        if voltage is not None and voltage > 0:
            state["turb_v"] = voltage
        elif args.range_distance_as_voltage and distance is not None:
            # 如果你把 RNGFND1_SCALING 设成 1 m/V，
            # 那么 distance 数值就可以当作电压使用。
            state["turb_v"] = distance

        state["turb_ntu"] = turbidity_voltage_to_ntu(state["turb_v"])
        state["last_turb_time"] = time.monotonic()

    elif msg_type == "DISTANCE_SENSOR":
        current_distance_cm = finite_or_none(getattr(msg, "current_distance", None))

        if current_distance_cm is not None:
            state["distance_sensor_m"] = current_distance_cm / 100.0
            state["last_distance_time"] = time.monotonic()

    elif msg_type in ["SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3"]:
        if args.pressure_message != "any" and msg_type != args.pressure_message:
            return

        pressure_mbar = finite_or_none(getattr(msg, "press_abs", None))
        temp_raw = finite_or_none(getattr(msg, "temperature", None))

        if pressure_mbar is None:
            return

        state["pressure_mbar"] = pressure_mbar

        if temp_raw is not None and temp_raw != 0:
            state["temp_c"] = temp_raw / 100.0

        if state["surface_pressure_mbar"] is None:
            state["surface_sum"] += pressure_mbar
            state["surface_count"] += 1

            if state["surface_count"] >= args.surface_samples:
                state["surface_pressure_mbar"] = (
                    state["surface_sum"] / state["surface_count"]
                )
                print(
                    f"已自动校准水面压力："
                    f"{state['surface_pressure_mbar']:.3f} mbar"
                )
        else:
            state["depth_m"] = pressure_to_depth_m(
                pressure_mbar=pressure_mbar,
                surface_pressure_mbar=state["surface_pressure_mbar"],
                density=args.density
            )

        state["last_pressure_time"] = time.monotonic()

    elif msg_type == "NAMED_VALUE_FLOAT":
        name = decode_named_value_name(getattr(msg, "name", ""))
        value = finite_or_none(getattr(msg, "value", None))

        if value is None:
            return

        if name == "TURB_V":
            state["turb_v"] = value
            state["turb_ntu"] = turbidity_voltage_to_ntu(value)
        elif name == "TURB_NTU":
            state["turb_ntu"] = value
        elif name == "DEPTH_M":
            state["depth_m"] = value
        elif name == "PRESS_MBAR":
            state["pressure_mbar"] = value
        elif name == "TEMP_C":
            state["temp_c"] = value


def drain_mavlink_messages(master, state, args):
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break

        process_message(msg, state, args)


def main():
    global running

    args = parse_args()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    headless = args.no_gui or not has_display

    master = connect_mavlink(args.connection, args.baud)
    request_all_message_intervals(master, args.hz)

    csv_file, csv_writer = prepare_csv(args.csv)
    plt, fig, axes, lines = setup_plot(headless)

    max_points = max(10, int(args.window / args.interval))

    xs = deque(maxlen=max_points)

    data_keys = [
        "turb_v",
        "turb_ntu",
        "rangefinder_distance_m",
        "distance_sensor_m",
        "pressure_mbar",
        "depth_m",
        "temp_c",
    ]

    data = {key: deque(maxlen=max_points) for key in data_keys}

    state = {
        "turb_v": None,
        "turb_ntu": None,
        "rangefinder_distance_m": None,
        "distance_sensor_m": None,
        "pressure_mbar": None,
        "depth_m": None,
        "temp_c": None,
        "surface_pressure_mbar": args.surface_pressure,
        "surface_sum": 0.0,
        "surface_count": 0,
        "last_message": None,
        "last_turb_time": None,
        "last_distance_time": None,
        "last_pressure_time": None,
    }

    start_time = time.monotonic()
    last_print_time = 0.0
    last_png_time = 0.0

    print("=" * 70)
    print("开始直接读取 Pixhawk MAVLink 数据")
    print("如果所有值一直是 None，说明 Pixhawk 没有发布对应消息，不是 RDK 程序没读到。")
    print(f"CSV 日志：{args.csv}")
    print(f"PNG 图像：{args.png}")
    print("按 Ctrl+C 退出")
    print("=" * 70)

    try:
        while running:
            now = time.monotonic()
            elapsed = now - start_time
            timestamp = datetime.now().isoformat(timespec="milliseconds")

            drain_mavlink_messages(master, state, args)

            xs.append(elapsed)

            for key in data_keys:
                data[key].append(value_or_nan(state[key]))

            csv_writer.writerow([
                timestamp,
                f"{elapsed:.3f}",
                fmt(state["turb_v"], 6),
                fmt(state["turb_ntu"], 6),
                fmt(state["rangefinder_distance_m"], 6),
                fmt(state["distance_sensor_m"], 6),
                fmt(state["pressure_mbar"], 6),
                fmt(state["surface_pressure_mbar"], 6),
                fmt(state["depth_m"], 6),
                fmt(state["temp_c"], 6),
                state["last_message"],
            ])
            csv_file.flush()

            update_plot(fig, axes, lines, xs, data, args.window)

            if elapsed - last_print_time >= args.print_every:
                print(
                    f"[{timestamp}] "
                    f"TURB_V={fmt(state['turb_v'])} V, "
                    f"TURB_NTU={fmt(state['turb_ntu'])}, "
                    f"DEPTH={fmt(state['depth_m'])} m, "
                    f"PRESS={fmt(state['pressure_mbar'])} mbar, "
                    f"TEMP={fmt(state['temp_c'])} C, "
                    f"RANGE_DIST={fmt(state['rangefinder_distance_m'])} m, "
                    f"DIST_SENSOR={fmt(state['distance_sensor_m'])} m, "
                    f"MSG={state['last_message']}"
                )
                last_print_time = elapsed

            if headless:
                if elapsed - last_png_time >= args.png_every:
                    fig.savefig(args.png, dpi=150, bbox_inches="tight")
                    last_png_time = elapsed
            else:
                plt.pause(0.001)

            time.sleep(args.interval)

    except Exception as e:
        print(f"运行出错：{e}")

    finally:
        try:
            fig.savefig(args.png, dpi=150, bbox_inches="tight")
            print(f"已保存图像：{args.png}")
        except Exception as e:
            print(f"保存图像失败：{e}")

        try:
            csv_file.close()
            print(f"已保存 CSV：{args.csv}")
        except Exception:
            pass

        print("程序已退出")


if __name__ == "__main__":
    main()
