#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
l08_test.py

L08 / L081MTW 水下超声波辅助感知测试程序。

功能：
1. L081MTW UART 标准距离读取
2. RS485 / Modbus 模式读取距离 + 回波距离 + 回波幅值
3. 单个超声波范围热力图
4. 回波可视化
5. 自动连续变化显示尺度
6. CSV 数据记录
7. 预留 Pixhawk 位姿地图累计接口

依赖：
    python3 -m pip install pyserial numpy matplotlib

UART 版本运行：
    python3 l08_test.py --port /dev/ttyUSB0 --protocol uart --raw

另一个传感器：
    python3 l08_test.py --port /dev/ttyUSB1 --protocol uart --raw

Modbus / RS485 版本运行：
    python3 l08_test.py --port /dev/ttyUSB0 --protocol modbus --address 1 --raw

注意：
    L081MTW 的 UART 标准输出只有距离帧：
        FF Data_H Data_L SUM

    所以 UART 模式下没有真实回波强度。
    若需要真实回波幅值，建议使用 RS485 / Modbus 版本读取 0x0126~0x0128。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


UART_HEADER = 0xFF
OUT_OF_WATER_VALUE = 0xFFFB


@dataclass
class Echo:
    distance_m: float
    amplitude: float
    label: str
    valid: bool = True


@dataclass
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0


@dataclass
class SonarReading:
    timestamp_s: float
    ok: bool
    distance_m: Optional[float]
    echoes: List[Echo] = field(default_factory=list)
    raw_hex: str = ""
    message: str = ""
    source: str = "unknown"
    strength_is_real: bool = False


def now_s() -> float:
    return time.time()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def bytes_to_hex(data: Optional[bytes]) -> str:
    if not data:
        return ""
    return " ".join(f"{b:02X}" for b in data)


def uart_checksum(data_h: int, data_l: int) -> int:
    return (UART_HEADER + data_h + data_l) & 0xFF


def find_uart_frame(raw: bytes) -> Optional[bytes]:
    """
    从串口原始数据中寻找合法 L081MTW UART 标准帧：
        FF Data_H Data_L SUM
    """
    if len(raw) < 4:
        return None

    for i in range(len(raw) - 3):
        if raw[i] != UART_HEADER:
            continue

        frame = raw[i:i + 4]
        _, data_h, data_l, checksum = frame

        if uart_checksum(data_h, data_l) == checksum:
            return frame

    return None


def parse_uart_frame(frame: bytes) -> Tuple[Optional[float], str]:
    if len(frame) != 4:
        return None, f"frame length error: {len(frame)}"

    header, data_h, data_l, checksum = frame

    if header != UART_HEADER:
        return None, f"header error: 0x{header:02X}"

    expected = uart_checksum(data_h, data_l)
    if checksum != expected:
        return None, f"checksum error: got 0x{checksum:02X}, expected 0x{expected:02X}"

    distance_mm = data_h * 256 + data_l

    if distance_mm == OUT_OF_WATER_VALUE:
        return None, "out of water value 0xFFFB"

    if distance_mm <= 0:
        return None, f"invalid distance {distance_mm} mm"

    return distance_mm / 1000.0, "uart distance"


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF

    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


def append_modbus_crc(payload: bytes) -> bytes:
    crc = modbus_crc16(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def verify_modbus_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False

    got = frame[-2] | (frame[-1] << 8)
    expected = modbus_crc16(frame[:-2])

    return got == expected


def u16_to_i16(value: int) -> int:
    if value & 0x8000:
        return value - 0x10000
    return value


class UartReader:
    """
    L081MTW UART 受控输出读取器。

    接线：
        USB-TTL TX -> L08 RX/B
        USB-TTL RX -> L08 TX/A
        GND        -> GND
        5V         -> VCC
    """

    def __init__(
        self,
        port: str,
        baudrate: int,
        trigger_low_ms: float,
        read_time_s: float,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.trigger_low_ms = trigger_low_ms
        self.read_time_s = read_time_s

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
        )

        self.ser.break_condition = False
        time.sleep(0.05)

    def close(self) -> None:
        try:
            self.ser.break_condition = False
        except Exception:
            pass

        try:
            self.ser.close()
        except Exception:
            pass

    def trigger(self) -> None:
        """
        通过 USB-TTL 的 TX break 拉低模块 RX，触发一次测距。
        """
        self.ser.break_condition = True
        time.sleep(self.trigger_low_ms / 1000.0)
        self.ser.break_condition = False

    def read_window(self) -> bytes:
        end = now_s() + self.read_time_s
        buf = bytearray()

        while now_s() < end:
            n = self.ser.in_waiting
            if n:
                buf.extend(self.ser.read(n))
            else:
                time.sleep(0.001)

        return bytes(buf)

    def read_once(self) -> SonarReading:
        try:
            self.ser.reset_input_buffer()
            self.trigger()
            raw = self.read_window()
        except Exception as exc:
            return SonarReading(
                timestamp_s=now_s(),
                ok=False,
                distance_m=None,
                raw_hex="",
                message=f"uart read error: {exc}",
                source="uart",
                strength_is_real=False,
            )

        raw_hex = bytes_to_hex(raw)

        if not raw:
            return SonarReading(
                timestamp_s=now_s(),
                ok=False,
                distance_m=None,
                raw_hex=raw_hex,
                message="no serial bytes received",
                source="uart",
                strength_is_real=False,
            )

        frame = find_uart_frame(raw)
        if frame is None:
            return SonarReading(
                timestamp_s=now_s(),
                ok=False,
                distance_m=None,
                raw_hex=raw_hex,
                message="bytes received, but no valid FF HH LL SUM frame",
                source="uart",
                strength_is_real=False,
            )

        distance_m, message = parse_uart_frame(frame)
        if distance_m is None:
            return SonarReading(
                timestamp_s=now_s(),
                ok=False,
                distance_m=None,
                raw_hex=raw_hex,
                message=message,
                source="uart",
                strength_is_real=False,
            )

        # UART 标准模式没有真实强度，这里用 amplitude=1.0 作为热力图占位置信度。
        return SonarReading(
            timestamp_s=now_s(),
            ok=True,
            distance_m=distance_m,
            echoes=[Echo(distance_m=distance_m, amplitude=1.0, label="distance", valid=True)],
            raw_hex=raw_hex,
            message=message,
            source="uart",
            strength_is_real=False,
        )


class ModbusReader:
    """
    RS485 / Modbus 读取器。

    读取：
        0x0101 实时距离
        0x0123 回波1距离
        0x0124 回波2距离
        0x0125 回波3距离
        0x0126 回波1幅值
        0x0127 回波2幅值
        0x0128 回波3幅值
    """

    def __init__(
        self,
        port: str,
        address: int,
        baudrate: int,
        response_timeout_s: float,
        modbus_gap_s: float,
    ) -> None:
        self.port = port
        self.address = address
        self.baudrate = baudrate
        self.response_timeout_s = response_timeout_s
        self.modbus_gap_s = modbus_gap_s

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
        )

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def read_registers(self, start_reg: int, count: int) -> Tuple[Optional[List[int]], bytes, str]:
        payload = bytes([
            self.address & 0xFF,
            0x03,
            (start_reg >> 8) & 0xFF,
            start_reg & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        ])

        request = append_modbus_crc(payload)
        expected_len = 1 + 1 + 1 + count * 2 + 2

        try:
            self.ser.reset_input_buffer()
            self.ser.write(request)
            self.ser.flush()

            end = now_s() + self.response_timeout_s
            raw = bytearray()

            while now_s() < end and len(raw) < expected_len:
                chunk = self.ser.read(expected_len - len(raw))
                if chunk:
                    raw.extend(chunk)
                else:
                    time.sleep(0.001)

            frame = bytes(raw)

            if len(frame) < 5:
                return None, frame, "modbus response too short"

            if frame[0] != (self.address & 0xFF):
                return None, frame, f"address mismatch: got 0x{frame[0]:02X}"

            if frame[1] & 0x80:
                return None, frame, f"modbus exception: 0x{frame[2]:02X}"

            if frame[1] != 0x03:
                return None, frame, f"function mismatch: 0x{frame[1]:02X}"

            if not verify_modbus_crc(frame):
                return None, frame, "crc error"

            byte_count = frame[2]
            if byte_count != count * 2:
                return None, frame, f"byte count mismatch: {byte_count}"

            values = []
            data = frame[3:3 + byte_count]
            for i in range(0, len(data), 2):
                values.append((data[i] << 8) | data[i + 1])

            return values, frame, "ok"

        except Exception as exc:
            return None, b"", f"modbus read error: {exc}"

    def read_once(self) -> SonarReading:
        values_distance, raw_distance, msg_distance = self.read_registers(0x0101, 1)
        time.sleep(self.modbus_gap_s)
        values_echo, raw_echo, msg_echo = self.read_registers(0x0123, 6)

        raw_hex = " | ".join(
            item for item in [bytes_to_hex(raw_distance), bytes_to_hex(raw_echo)] if item
        )

        if values_distance is None:
            return SonarReading(
                timestamp_s=now_s(),
                ok=False,
                distance_m=None,
                raw_hex=raw_hex,
                message=f"distance register failed: {msg_distance}",
                source="modbus",
                strength_is_real=True,
            )

        distance_mm = u16_to_i16(values_distance[0])

        if distance_mm == OUT_OF_WATER_VALUE:
            return SonarReading(
                timestamp_s=now_s(),
                ok=False,
                distance_m=None,
                raw_hex=raw_hex,
                message="out of water value 0xFFFB",
                source="modbus",
                strength_is_real=True,
            )

        distance_m = distance_mm / 1000.0 if distance_mm > 0 else None
        echoes: List[Echo] = []

        if values_echo is not None and len(values_echo) >= 6:
            echo_distances_mm = values_echo[:3]
            echo_amplitudes = values_echo[3:6]

            for idx in range(3):
                d_mm = echo_distances_mm[idx]
                amp = float(echo_amplitudes[idx])
                valid = 0 < d_mm < 0xFFFB

                echoes.append(
                    Echo(
                        distance_m=d_mm / 1000.0 if valid else 0.0,
                        amplitude=amp,
                        label=f"echo{idx + 1}",
                        valid=valid,
                    )
                )

            message = "modbus distance + echo amplitudes"
        else:
            message = f"distance ok, echo registers failed: {msg_echo}"
            if distance_m is not None:
                echoes.append(Echo(distance_m=distance_m, amplitude=1.0, label="distance", valid=True))

        return SonarReading(
            timestamp_s=now_s(),
            ok=distance_m is not None,
            distance_m=distance_m,
            echoes=echoes,
            raw_hex=raw_hex,
            message=message,
            source="modbus",
            strength_is_real=True,
        )


class CsvLogger:
    def __init__(self, path: str) -> None:
        self.path = path
        self.file = None
        self.writer = None

        if not self.path:
            return

        folder = os.path.dirname(os.path.abspath(self.path))
        if folder:
            os.makedirs(folder, exist_ok=True)

        self.file = open(self.path, "a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)

        if os.path.getsize(self.path) == 0:
            self.writer.writerow([
                "timestamp_s",
                "source",
                "ok",
                "distance_m",
                "strength_is_real",
                "echo1_distance_m",
                "echo1_amplitude",
                "echo2_distance_m",
                "echo2_amplitude",
                "echo3_distance_m",
                "echo3_amplitude",
                "view_range_m",
                "raw_hex",
                "message",
            ])
            self.file.flush()

    def close(self) -> None:
        if self.file:
            self.file.close()

    def write(self, reading: SonarReading, view_range_m: float) -> None:
        if not self.writer:
            return

        echo_columns = []
        for idx in range(3):
            if idx < len(reading.echoes) and reading.echoes[idx].valid:
                echo_columns.extend([
                    f"{reading.echoes[idx].distance_m:.4f}",
                    f"{reading.echoes[idx].amplitude:.3f}",
                ])
            else:
                echo_columns.extend(["", ""])

        self.writer.writerow([
            f"{reading.timestamp_s:.6f}",
            reading.source,
            int(reading.ok),
            "" if reading.distance_m is None else f"{reading.distance_m:.4f}",
            int(reading.strength_is_real),
            *echo_columns,
            f"{view_range_m:.4f}",
            reading.raw_hex,
            reading.message,
        ])
        self.file.flush()


def sonar_echo_points_in_world(
    reading: SonarReading,
    pose: Pose2D,
    sensor_yaw_offset_rad: float,
) -> List[Tuple[float, float, Echo]]:
    """
    Pixhawk 地图累计接口。

    后面接 Pixhawk 后，把 pose.x_m、pose.y_m、pose.yaw_rad 更新为真实位姿，
    这里返回的就是每个回波点在世界坐标系中的位置。
    """
    points = []
    yaw = pose.yaw_rad + sensor_yaw_offset_rad

    for echo in reading.echoes:
        if not echo.valid:
            continue

        x = pose.x_m + echo.distance_m * math.cos(yaw)
        y = pose.y_m + echo.distance_m * math.sin(yaw)
        points.append((x, y, echo))

    return points


class AdaptiveRangeController:
    """
    连续自动尺度控制器，不是三档切换。

    逻辑：
    - 目标越远，显示范围越大。
    - 扩大速度快，避免目标出界。
    - 缩小速度慢，避免画面抖动。
    """

    def __init__(
        self,
        initial_range_m: float,
        min_view_m: float,
        max_view_m: float,
        margin_ratio: float,
        smooth_alpha: float,
        shrink_delay_s: float,
    ) -> None:
        self.current_range_m = clamp(initial_range_m, min_view_m, max_view_m)
        self.min_view_m = min_view_m
        self.max_view_m = max_view_m
        self.margin_ratio = margin_ratio
        self.smooth_alpha = clamp(smooth_alpha, 0.01, 1.0)
        self.shrink_delay_s = shrink_delay_s
        self.last_expand_time_s = now_s()

    def get_relevant_distance(self, reading: SonarReading) -> Optional[float]:
        distances = []

        if reading.distance_m is not None and reading.distance_m > 0:
            distances.append(reading.distance_m)

        for echo in reading.echoes:
            if echo.valid and echo.distance_m > 0:
                distances.append(echo.distance_m)

        if not distances:
            return None

        return max(distances)

    def update(self, reading: SonarReading) -> Tuple[float, bool]:
        distance = self.get_relevant_distance(reading)
        if distance is None:
            return self.current_range_m, False

        target = distance * (1.0 + self.margin_ratio)
        target = max(target, self.min_view_m)

        if distance > self.current_range_m * 0.82:
            target = max(target, distance * 1.65)
            self.last_expand_time_s = now_s()

        target = clamp(target, self.min_view_m, self.max_view_m)

        if target > self.current_range_m:
            alpha = max(self.smooth_alpha, 0.45)
        else:
            if now_s() - self.last_expand_time_s < self.shrink_delay_s:
                return self.current_range_m, False
            alpha = min(self.smooth_alpha, 0.10)

        new_range = (1.0 - alpha) * self.current_range_m + alpha * target
        new_range = clamp(new_range, self.min_view_m, self.max_view_m)

        changed = abs(new_range - self.current_range_m) / max(self.current_range_m, 1e-6) > 0.03
        self.current_range_m = new_range

        return self.current_range_m, changed


class SonarGui:
    def __init__(
        self,
        reader,
        logger: CsvLogger,
        sensor_min_m: float,
        sensor_max_m: float,
        beam_deg: float,
        initial_view_range_m: float,
        auto_scale: bool,
        min_auto_view_m: float,
        auto_margin_ratio: float,
        auto_smooth_alpha: float,
        auto_shrink_delay_s: float,
        grid_size: int,
        heat_decay: float,
        history_len: int,
        sensor_yaw_offset_deg: float,
        raw_print: bool,
        update_interval_ms: int,
    ) -> None:
        self.reader = reader
        self.logger = logger

        self.sensor_min_m = sensor_min_m
        self.sensor_max_m = sensor_max_m
        self.beam_deg = beam_deg
        self.beam_rad = math.radians(beam_deg)

        self.view_range_m = clamp(initial_view_range_m, min_auto_view_m, sensor_max_m)
        self.auto_scale = auto_scale

        self.range_controller = AdaptiveRangeController(
            initial_range_m=self.view_range_m,
            min_view_m=min_auto_view_m,
            max_view_m=sensor_max_m,
            margin_ratio=auto_margin_ratio,
            smooth_alpha=auto_smooth_alpha,
            shrink_delay_s=auto_shrink_delay_s,
        )

        self.grid_size = grid_size
        self.heat_decay = heat_decay
        self.history_len = history_len
        self.sensor_yaw_offset_rad = math.radians(sensor_yaw_offset_deg)
        self.raw_print = raw_print

        self.pose = Pose2D()

        self.x_grid = None
        self.y_grid = None
        self.r_grid = None
        self.theta_grid = None
        self.heatmap = None
        self.extent = None

        self.distances: List[float] = []
        self.strengths: List[float] = []
        self.running = True

        self._build_grid()

        self.fig = plt.figure(figsize=(13, 8))
        gs = self.fig.add_gridspec(2, 3, width_ratios=[1.65, 1.0, 1.0])
        self.ax_heat = self.fig.add_subplot(gs[:, 0])
        self.ax_echo = self.fig.add_subplot(gs[0, 1])
        self.ax_history = self.fig.add_subplot(gs[1, 1])
        self.ax_status = self.fig.add_subplot(gs[:, 2])

        self.im = None
        self.cone_lines = []
        self.current_point = None
        self.echo_line = None
        self.distance_line = None
        self.strength_line = None
        self.status_text = None

        self._setup_plot()
        self._connect_events()

        self.anim = FuncAnimation(
            self.fig,
            self._update,
            interval=update_interval_ms,
            blit=False,
            cache_frame_data=False,
        )

    def _build_grid(self) -> None:
        lateral = self.view_range_m * math.sin(max(self.beam_rad, math.radians(25.0))) * 1.25
        lateral = max(lateral, self.view_range_m * 0.22, 0.08)

        x = np.linspace(0.0, self.view_range_m, self.grid_size)
        y = np.linspace(-lateral, lateral, self.grid_size)

        self.x_grid, self.y_grid = np.meshgrid(x, y)
        self.r_grid = np.sqrt(self.x_grid ** 2 + self.y_grid ** 2)
        self.theta_grid = np.arctan2(self.y_grid, self.x_grid + 1e-9)
        self.extent = [0.0, self.view_range_m, -lateral, lateral]
        self.heatmap = np.zeros_like(self.x_grid, dtype=float)

    def _setup_plot(self) -> None:
        self.fig.suptitle("L08 single-sonar auxiliary perception")

        self.ax_heat.set_title("Adaptive single-sonar heatmap")
        self.ax_heat.set_xlabel("Forward range / m")
        self.ax_heat.set_ylabel("Lateral range / m")
        self.ax_heat.set_xlim(self.extent[0], self.extent[1])
        self.ax_heat.set_ylim(self.extent[2], self.extent[3])
        self.ax_heat.set_aspect("equal", adjustable="box")

        self.im = self.ax_heat.imshow(
            self.heatmap,
            origin="lower",
            extent=self.extent,
            interpolation="nearest",
            vmin=0.0,
            vmax=1.0,
            aspect="auto",
        )

        self.current_point, = self.ax_heat.plot([], [], marker="o", linestyle="None")
        self._draw_beam()

        self.ax_echo.set_title("Echo distance / strength")
        self.ax_echo.set_xlabel("Distance / m")
        self.ax_echo.set_ylabel("Amplitude or confidence")
        self.ax_echo.set_xlim(0.0, self.view_range_m)
        self.ax_echo.set_ylim(0.0, 1.2)
        self.echo_line, = self.ax_echo.plot([], [], marker="o", linestyle="-")

        self.ax_history.set_title("History")
        self.ax_history.set_xlabel("Sample")
        self.ax_history.set_ylabel("Value")
        self.ax_history.set_xlim(0, self.history_len)
        self.ax_history.set_ylim(0.0, max(1.0, self.view_range_m))
        self.distance_line, = self.ax_history.plot([], [], label="distance m")
        self.strength_line, = self.ax_history.plot([], [], label="strength normalized")
        self.ax_history.legend(loc="upper right")

        self.ax_status.axis("off")
        self.status_text = self.ax_status.text(
            0.02,
            0.98,
            "",
            transform=self.ax_status.transAxes,
            ha="left",
            va="top",
            family="monospace",
        )

        self.fig.tight_layout()

    def _draw_beam(self) -> None:
        for line in self.cone_lines:
            try:
                line.remove()
            except Exception:
                pass

        self.cone_lines = []
        half = self.beam_rad / 2.0

        for angle in [-half, 0.0, half]:
            x_end = self.view_range_m * math.cos(angle)
            y_end = self.view_range_m * math.sin(angle)
            line, = self.ax_heat.plot([0.0, x_end], [0.0, y_end], linestyle="--", linewidth=1)
            self.cone_lines.append(line)

        arc_angles = np.linspace(-half, half, 120)
        arc_x = self.view_range_m * np.cos(arc_angles)
        arc_y = self.view_range_m * np.sin(arc_angles)
        arc_line, = self.ax_heat.plot(arc_x, arc_y, linestyle="--", linewidth=1)
        self.cone_lines.append(arc_line)

    def _connect_events(self) -> None:
        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)
        self.fig.canvas.mpl_connect("close_event", self._on_close)

    def _on_close(self, _event) -> None:
        self.running = False
        try:
            self.reader.close()
        except Exception:
            pass
        try:
            self.logger.close()
        except Exception:
            pass

    def _on_key_press(self, event) -> None:
        if event.key in ["q", "escape"]:
            self.running = False
            plt.close(self.fig)
            return

        if event.key == "r":
            self.distances.clear()
            self.strengths.clear()
            self.heatmap[:] = 0.0
            return

        if event.key == "a":
            self.auto_scale = not self.auto_scale
            return

        if event.key in ["+", "="]:
            self.auto_scale = False
            self.set_view_range(self.view_range_m * 1.25)
            return

        if event.key in ["-", "_"]:
            self.auto_scale = False
            self.set_view_range(self.view_range_m / 1.25)
            return

    def set_view_range(self, new_range_m: float) -> None:
        new_range_m = clamp(new_range_m, self.range_controller.min_view_m, self.sensor_max_m)

        if abs(new_range_m - self.view_range_m) / max(self.view_range_m, 1e-6) < 0.01:
            return

        self.view_range_m = new_range_m
        self.range_controller.current_range_m = new_range_m

        self._build_grid()

        self.im.set_data(self.heatmap)
        self.im.set_extent(self.extent)

        self.ax_heat.set_xlim(self.extent[0], self.extent[1])
        self.ax_heat.set_ylim(self.extent[2], self.extent[3])
        self.ax_echo.set_xlim(0.0, self.view_range_m)
        self.ax_history.set_ylim(0.0, max(1.0, self.view_range_m))
        self._draw_beam()
        self.fig.canvas.draw_idle()

    def normalize_strength(self, echo: Echo, reading: SonarReading) -> float:
        if not echo.valid:
            return 0.0

        if reading.strength_is_real:
            return clamp(echo.amplitude / 1000.0, 0.05, 1.0)

        return 0.65

    def maybe_auto_scale(self, reading: SonarReading) -> None:
        if not self.auto_scale:
            return

        new_range, changed = self.range_controller.update(reading)
        if changed:
            self.set_view_range(new_range)

    def update_heatmap(self, reading: SonarReading) -> None:
        self.heatmap *= self.heat_decay

        if not reading.ok:
            return

        half = self.beam_rad / 2.0
        inside_beam = np.abs(self.theta_grid) <= half
        inside_range = (self.r_grid >= self.sensor_min_m) & (self.r_grid <= self.sensor_max_m)
        valid_region = inside_beam & inside_range

        sigma_theta = max(self.beam_rad / 4.0, math.radians(2.0))

        for echo in reading.echoes:
            if not echo.valid:
                continue

            d = echo.distance_m
            if d <= 0 or d > self.sensor_max_m:
                continue

            strength = self.normalize_strength(echo, reading)
            sigma_r = max(0.012, d * 0.035)

            ring = np.exp(-0.5 * ((self.r_grid - d) / sigma_r) ** 2)
            angular = np.exp(-0.5 * (self.theta_grid / sigma_theta) ** 2)
            contribution = strength * ring * angular * valid_region
            self.heatmap = np.maximum(self.heatmap, contribution)

        np.clip(self.heatmap, 0.0, 1.0, out=self.heatmap)

    def update_history(self, reading: SonarReading) -> None:
        if not reading.ok or reading.distance_m is None:
            return

        self.distances.append(reading.distance_m)

        if reading.echoes:
            self.strengths.append(self.normalize_strength(reading.echoes[0], reading))
        else:
            self.strengths.append(0.0)

        if len(self.distances) > self.history_len:
            self.distances = self.distances[-self.history_len:]
            self.strengths = self.strengths[-self.history_len:]

    def format_status(self, reading: SonarReading) -> str:
        if reading.distance_m is None:
            distance_text = "-"
        else:
            distance_text = f"{reading.distance_m:.3f} m / {reading.distance_m * 100.0:.1f} cm"

        strength_type = "real Modbus echo amplitude" if reading.strength_is_real else "UART placeholder confidence"

        echo_lines = []
        for idx in range(3):
            if idx < len(reading.echoes) and reading.echoes[idx].valid:
                echo = reading.echoes[idx]
                if reading.strength_is_real:
                    amp_text = f"{echo.amplitude:.1f} mV"
                else:
                    amp_text = f"{echo.amplitude:.2f}"
                echo_lines.append(f"  {echo.label}: {echo.distance_m:.3f} m, amp={amp_text}")
            else:
                echo_lines.append(f"  echo{idx + 1}: -")

        world_points = sonar_echo_points_in_world(
            reading=reading,
            pose=self.pose,
            sensor_yaw_offset_rad=self.sensor_yaw_offset_rad,
        )

        if world_points:
            world_text = f"({world_points[0][0]:.3f}, {world_points[0][1]:.3f})"
        else:
            world_text = "-"

        lines = [
            "Controls:",
            "  a: auto scale on/off",
            "  +/-: manual zoom, disables auto",
            "  r: reset heat/history",
            "  q: quit",
            "",
            f"Protocol: {reading.source}",
            f"OK: {reading.ok}",
            f"Distance: {distance_text}",
            f"View range: {self.view_range_m:.3f} m",
            f"Auto scale: {self.auto_scale}",
            f"Sensor range: {self.sensor_min_m:.2f} ~ {self.sensor_max_m:.2f} m",
            f"Beam angle: {self.beam_deg:.1f} deg",
            f"Strength type: {strength_type}",
            "",
            "Echoes:",
            *echo_lines,
            "",
            "Pixhawk/world map placeholder:",
            f"  pose=({self.pose.x_m:.2f}, {self.pose.y_m:.2f}), yaw={math.degrees(self.pose.yaw_rad):.1f} deg",
            f"  first echo world point: {world_text}",
            "",
            f"Message: {reading.message}",
            "",
            "Raw:",
            reading.raw_hex[:360] if reading.raw_hex else "-",
        ]

        return "\n".join(lines)

    def _update(self, _frame_idx):
        if not self.running:
            return []

        reading = self.reader.read_once()
        self.maybe_auto_scale(reading)
        self.update_heatmap(reading)
        self.update_history(reading)
        self.logger.write(reading, self.view_range_m)

        if self.raw_print:
            print(
                f"[{time.strftime('%H:%M:%S')}] "
                f"ok={reading.ok} "
                f"distance={reading.distance_m} "
                f"view={self.view_range_m:.3f}m "
                f"raw={reading.raw_hex if reading.raw_hex else 'None'} "
                f"msg={reading.message}"
            )

        self.im.set_data(self.heatmap)

        if reading.ok and reading.distance_m is not None:
            self.current_point.set_data([reading.distance_m], [0.0])
        else:
            self.current_point.set_data([], [])

        valid_echoes = [echo for echo in reading.echoes if echo.valid]
        if valid_echoes:
            x = [echo.distance_m for echo in valid_echoes]
            y = [self.normalize_strength(echo, reading) for echo in valid_echoes]
            self.echo_line.set_data(x, y)
            self.ax_echo.set_ylim(0.0, max(1.2, max(y) * 1.2))
        else:
            self.echo_line.set_data([], [])

        sample_x = list(range(len(self.distances)))
        self.distance_line.set_data(sample_x, self.distances)
        self.strength_line.set_data(sample_x, self.strengths)

        self.ax_history.set_xlim(0, max(10, len(sample_x)))
        y_max = max(1.0, self.view_range_m)
        if self.distances:
            y_max = max(y_max, max(self.distances) * 1.15)
        self.ax_history.set_ylim(0.0, y_max)

        self.status_text.set_text(self.format_status(reading))

        return [
            self.im,
            self.current_point,
            self.echo_line,
            self.distance_line,
            self.strength_line,
            self.status_text,
        ]

    def show(self) -> None:
        plt.show()


def create_reader(args):
    if args.protocol == "uart":
        return UartReader(
            port=args.port,
            baudrate=args.baudrate,
            trigger_low_ms=args.trigger_low_ms,
            read_time_s=args.read_time,
        )

    if args.protocol == "modbus":
        return ModbusReader(
            port=args.port,
            address=args.address,
            baudrate=args.baudrate,
            response_timeout_s=args.read_time,
            modbus_gap_s=args.modbus_gap,
        )

    raise ValueError(f"unknown protocol: {args.protocol}")


def parse_args(argv: Optional[Sequence[str]] = None):
    parser = argparse.ArgumentParser(description="L08 adaptive single-sonar GUI")

    parser.add_argument("--port", "-p", required=True, help="串口，例如 /dev/ttyUSB0、/dev/ttyUSB1、COM3")
    parser.add_argument("--protocol", choices=["uart", "modbus"], default="uart", help="uart 或 modbus")
    parser.add_argument("--baudrate", "-b", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument("--address", type=int, default=1, help="Modbus 地址，默认 1")
    parser.add_argument("--trigger-low-ms", type=float, default=35.0, help="UART 触发低电平时间 ms")
    parser.add_argument("--read-time", type=float, default=0.28, help="读取窗口秒")
    parser.add_argument("--modbus-gap", type=float, default=0.02, help="Modbus 寄存器读取间隔秒")

    parser.add_argument("--sensor-min-m", type=float, default=0.05, help="传感器最小有效距离 m")
    parser.add_argument("--sensor-max-m", type=float, default=8.0, help="传感器最大显示距离 m")
    parser.add_argument("--beam-deg", type=float, default=15.0, help="探测角度")
    parser.add_argument("--view-range-m", type=float, default=0.30, help="初始显示范围 m")

    parser.add_argument("--no-auto-scale", action="store_true", help="关闭自动连续尺度")
    parser.add_argument("--min-auto-view-m", type=float, default=0.25, help="自动尺度最小窗口 m")
    parser.add_argument("--auto-margin-ratio", type=float, default=0.35, help="自动尺度边距比例")
    parser.add_argument("--auto-smooth-alpha", type=float, default=0.22, help="自动尺度平滑系数")
    parser.add_argument("--auto-shrink-delay-s", type=float, default=1.5, help="自动缩小等待时间秒")

    parser.add_argument("--grid-size", type=int, default=180, help="热力图网格大小")
    parser.add_argument("--heat-decay", type=float, default=0.90, help="热力图衰减系数")
    parser.add_argument("--history-len", type=int, default=150, help="历史长度")
    parser.add_argument("--sensor-yaw-offset-deg", type=float, default=0.0, help="传感器安装偏角，预留 Pixhawk 用")
    parser.add_argument("--update-interval-ms", type=int, default=80, help="界面刷新间隔 ms")
    parser.add_argument("--log-csv", default="", help="CSV 日志路径")
    parser.add_argument("--raw", action="store_true", help="打印原始串口数据")

    return parser.parse_args(argv)


def install_signal_handlers(reader_holder, logger_holder):
    def handle_signal(_signum, _frame):
        try:
            if reader_holder[0] is not None:
                reader_holder[0].close()
        except Exception:
            pass

        try:
            if logger_holder[0] is not None:
                logger_holder[0].close()
        except Exception:
            pass

        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    reader_holder = [None]
    logger_holder = [None]
    install_signal_handlers(reader_holder, logger_holder)

    try:
        reader = create_reader(args)
        logger = CsvLogger(args.log_csv)

        reader_holder[0] = reader
        logger_holder[0] = logger

        auto_scale = not args.no_auto_scale

        print("L08 adaptive sonar GUI started")
        print("  file name: l08_test.py")
        print(f"  port: {args.port}")
        print(f"  protocol: {args.protocol}")
        print(f"  baudrate: {args.baudrate}")
        print(f"  initial view range: {args.view_range_m:.3f} m")
        print(f"  auto scale: {auto_scale}")
        print(f"  sensor range: {args.sensor_min_m:.3f} ~ {args.sensor_max_m:.3f} m")
        print(f"  beam angle: {args.beam_deg:.1f} deg")
        print("  keys: a=auto on/off, +/-=manual zoom, r=reset, q=quit")
        print()

        gui = SonarGui(
            reader=reader,
            logger=logger,
            sensor_min_m=args.sensor_min_m,
            sensor_max_m=args.sensor_max_m,
            beam_deg=args.beam_deg,
            initial_view_range_m=args.view_range_m,
            auto_scale=auto_scale,
            min_auto_view_m=args.min_auto_view_m,
            auto_margin_ratio=args.auto_margin_ratio,
            auto_smooth_alpha=args.auto_smooth_alpha,
            auto_shrink_delay_s=args.auto_shrink_delay_s,
            grid_size=args.grid_size,
            heat_decay=args.heat_decay,
            history_len=args.history_len,
            sensor_yaw_offset_deg=args.sensor_yaw_offset_deg,
            raw_print=args.raw,
            update_interval_ms=args.update_interval_ms,
        )

        gui.show()

        reader.close()
        logger.close()

        return 0

    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        print("请检查串口号、权限、接线、是否被其他程序占用。", file=sys.stderr)
        return 2

    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
