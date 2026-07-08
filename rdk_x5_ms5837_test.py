#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rdk_x5_ms5837_test.py

RDK X5 直接通过 I2C 读取 MS5837-02BA / MS5837-30BA 压力传感器。

接线：
    MS5837 VCC -> RDK X5 3.3V
    MS5837 GND -> RDK X5 GND
    MS5837 SDA -> RDK X5 物理 3 脚，I2C5_SDA
    MS5837 SCL -> RDK X5 物理 5 脚，I2C5_SCL

安装依赖：
    sudo apt update
    sudo apt install -y i2c-tools python3-smbus
    python3 -m pip install smbus2

先查看 I2C：
    ls /dev/i2c-*
    sudo i2cdetect -y 5

如果 /dev/i2c-5 不存在，就先运行本程序自动扫描：
    python3 rdk_x5_ms5837_test.py --scan

运行：
    python3 rdk_x5_ms5837_test.py

指定总线：
    python3 rdk_x5_ms5837_test.py --bus 5

指定地址：
    python3 rdk_x5_ms5837_test.py --bus 5 --addr 0x76
"""

import argparse
import glob
import time

try:
    import smbus2
except Exception:
    smbus2 = None


MS5837_ADDR_1 = 0x76
MS5837_ADDR_2 = 0x77

CMD_RESET = 0x1E
CMD_ADC_READ = 0x00
CMD_CONVERT_D1_8192 = 0x4A
CMD_CONVERT_D2_8192 = 0x5A
CMD_PROM_READ_BASE = 0xA0

WATER_DENSITY_KG_M3 = 997.0
GRAVITY_M_S2 = 9.80665


class MS5837:
    def __init__(self, bus_id, address=0x76):
        if smbus2 is None:
            raise RuntimeError("缺少 smbus2，请安装：python3 -m pip install smbus2")

        self.bus_id = int(bus_id)
        self.address = int(address)
        self.bus = smbus2.SMBus(self.bus_id)

        self.C = [0] * 8
        self.pressure_mbar = None
        self.temperature_c = None

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def reset(self):
        self.bus.write_byte(self.address, CMD_RESET)
        time.sleep(0.02)

    def read_prom(self):
        values = []

        for i in range(8):
            command = CMD_PROM_READ_BASE + i * 2
            data = self.bus.read_i2c_block_data(self.address, command, 2)
            value = (data[0] << 8) | data[1]
            values.append(value)

        self.C = values

        if self.C[0] == 0 and self.C[1] == 0:
            raise RuntimeError("PROM 数据异常，全是 0，可能没有接好或地址错误。")

        return values

    def read_adc(self):
        data = self.bus.read_i2c_block_data(self.address, CMD_ADC_READ, 3)
        return (data[0] << 16) | (data[1] << 8) | data[2]

    def convert_and_read_adc(self, command):
        self.bus.write_byte(self.address, command)
        time.sleep(0.02)
        return self.read_adc()

    def read_raw(self):
        D1 = self.convert_and_read_adc(CMD_CONVERT_D1_8192)
        D2 = self.convert_and_read_adc(CMD_CONVERT_D2_8192)
        return D1, D2

    def read(self):
        """
        MS5837 官方补偿公式。

        输出：
            pressure_mbar: mbar / hPa
            temperature_c: 摄氏度
        """
        D1, D2 = self.read_raw()

        C = self.C

        dT = D2 - C[5] * 256
        TEMP = 2000 + dT * C[6] / 8388608.0

        OFF = C[2] * 65536.0 + C[4] * dT / 128.0
        SENS = C[1] * 32768.0 + C[3] * dT / 256.0

        # 二阶温度补偿
        Ti = 0.0
        OFFi = 0.0
        SENSi = 0.0

        if TEMP < 2000:
            Ti = 3.0 * dT * dT / 8589934592.0
            OFFi = 3.0 * (TEMP - 2000.0) * (TEMP - 2000.0) / 2.0
            SENSi = 5.0 * (TEMP - 2000.0) * (TEMP - 2000.0) / 8.0

            if TEMP < -1500:
                OFFi += 7.0 * (TEMP + 1500.0) * (TEMP + 1500.0)
                SENSi += 4.0 * (TEMP + 1500.0) * (TEMP + 1500.0)

        TEMP2 = TEMP - Ti
        OFF2 = OFF - OFFi
        SENS2 = SENS - SENSi

        # P 单位：0.01 mbar
        P = (D1 * SENS2 / 2097152.0 - OFF2) / 8192.0

        self.temperature_c = TEMP2 / 100.0
        self.pressure_mbar = P / 10.0

        return self.pressure_mbar, self.temperature_c, D1, D2

    def depth_m(self, surface_pressure_mbar):
        if self.pressure_mbar is None:
            return None

        pressure_diff_pa = (self.pressure_mbar - surface_pressure_mbar) * 100.0
        return pressure_diff_pa / (WATER_DENSITY_KG_M3 * GRAVITY_M_S2)


def list_i2c_buses():
    buses = []

    for path in sorted(glob.glob("/dev/i2c-*")):
        try:
            bus_id = int(path.split("-")[-1])
            buses.append(bus_id)
        except Exception:
            pass

    return buses


def probe_address(bus_id, address):
    try:
        bus = smbus2.SMBus(bus_id)
        bus.write_byte(address, CMD_RESET)
        time.sleep(0.02)

        data = bus.read_i2c_block_data(address, CMD_PROM_READ_BASE, 2)
        bus.close()

        value = (data[0] << 8) | data[1]

        if value == 0x0000 or value == 0xFFFF:
            return False

        return True

    except Exception:
        return False


def scan_ms5837():
    print("正在扫描 I2C 总线...")

    buses = list_i2c_buses()

    if not buses:
        print("没有找到 /dev/i2c-*。请确认 I2C 已启用。")
        return []

    found = []

    for bus_id in buses:
        for address in [MS5837_ADDR_1, MS5837_ADDR_2]:
            ok = probe_address(bus_id, address)

            if ok:
                print(f"发现疑似 MS5837：/dev/i2c-{bus_id}, address=0x{address:02X}")
                found.append((bus_id, address))

    if not found:
        print("没有发现 MS5837。请检查接线、电源、I2C 是否启用、地址是否为 0x76/0x77。")

    return found


def main():
    parser = argparse.ArgumentParser(description="RDK X5 MS5837 I2C 测试程序")

    parser.add_argument(
        "--bus",
        type=int,
        default=None,
        help="I2C 总线编号，例如 5 表示 /dev/i2c-5",
    )

    parser.add_argument(
        "--addr",
        type=str,
        default="0x76",
        help="I2C 地址，常见 0x76 或 0x77",
    )

    parser.add_argument(
        "--scan",
        action="store_true",
        help="自动扫描所有 /dev/i2c-* 上的 0x76/0x77",
    )

    args = parser.parse_args()

    if smbus2 is None:
        print("缺少 smbus2，请先安装：")
        print("python3 -m pip install smbus2")
        return

    if args.scan:
        found = scan_ms5837()

        if not found:
            return

        bus_id, address = found[0]
        print(f"自动使用第一个设备：/dev/i2c-{bus_id}, address=0x{address:02X}")
    else:
        if args.bus is None:
            # RDK X5 表里物理 3/5 是 I2C5_SDA/SCL，所以默认先用 bus 5
            bus_id = 5
        else:
            bus_id = int(args.bus)

        address = int(args.addr, 16) if args.addr.lower().startswith("0x") else int(args.addr)

    sensor = MS5837(bus_id=bus_id, address=address)

    try:
        print(f"打开 MS5837：/dev/i2c-{bus_id}, address=0x{address:02X}")

        sensor.reset()
        prom = sensor.read_prom()

        print("PROM 系数：")
        for i, value in enumerate(prom):
            print(f"  C{i} = {value}")

        print()
        print("开始读取压力。前 3 秒作为水面压力校零参考。")
        print("按 Ctrl+C 退出。")
        print()

        surface_pressure = None
        samples = []

        start_time = time.time()

        while True:
            pressure_mbar, temperature_c, D1, D2 = sensor.read()

            if time.time() - start_time < 3.0:
                samples.append(pressure_mbar)

                if samples:
                    surface_pressure = sum(samples) / len(samples)

            if surface_pressure is None:
                depth = 0.0
            else:
                depth = sensor.depth_m(surface_pressure)

            print(
                f"pressure={pressure_mbar:9.3f} hPa, "
                f"temp={temperature_c:7.3f} C, "
                f"depth={depth:7.3f} m, "
                f"zero={surface_pressure if surface_pressure else 0:9.3f} hPa, "
                f"D1={D1}, D2={D2}"
            )

            time.sleep(0.5)

    except KeyboardInterrupt:
        print()
        print("退出。")

    finally:
        sensor.close()


if __name__ == "__main__":
    main()
