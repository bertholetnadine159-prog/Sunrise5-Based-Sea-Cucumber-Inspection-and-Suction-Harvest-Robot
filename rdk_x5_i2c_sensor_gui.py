#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rdk_x5_sensor_read_all.py

RDK X5 同时读取：
1. VEML7700-1：默认 /dev/i2c-0，addr 0x10，物理 27/28
2. VEML7700-2：默认 /dev/i2c-5，addr 0x10，物理 3/5
3. MS5837-02BA：自动扫描所有 /dev/i2c-* 的 0x76 / 0x77

特点：
- 不需要 smbus
- 不需要 smbus2
- 不需要 pip
- 不需要联网
- 直接使用 i2c-tools：i2cget / i2cset / i2ctransfer

依赖：
    sudo apt install -y i2c-tools

运行：
    sudo python3 rdk_x5_sensor_read_all.py

如果要指定 MS5837 总线：
    sudo python3 rdk_x5_sensor_read_all.py --ms-bus 0 --ms-addr 0x76

如果 VEML7700 总线不一样：
    sudo python3 rdk_x5_sensor_read_all.py --veml1-bus 0 --veml2-bus 5
"""

import argparse
import glob
import shutil
import subprocess
import time
from datetime import datetime


VEML7700_ADDR = 0x10
VEML7700_REG_ALS_CONF = 0x00
VEML7700_REG_ALS = 0x04
VEML7700_REG_WHITE = 0x05

MS5837_ADDR_LIST = [0x76, 0x77]

MS5837_CMD_RESET = 0x1E
MS5837_CMD_ADC_READ = 0x00
MS5837_CMD_CONVERT_D1_8192 = 0x4A
MS5837_CMD_CONVERT_D2_8192 = 0x5A
MS5837_CMD_PROM_READ_BASE = 0xA0

WATER_DENSITY_KG_M3 = 997.0
GRAVITY_M_S2 = 9.80665


def parse_int(value):
    value = str(value).strip()

    if value.lower().startswith("0x"):
        return int(value, 16)

    return int(value)


def run_cmd(cmd):
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=True,
    )

    return result.stdout.strip()


def list_i2c_buses():
    buses = []

    for path in sorted(glob.glob("/dev/i2c-*")):
        try:
            buses.append(int(path.split("-")[-1]))
        except Exception:
            pass

    return buses


def check_tools():
    missing = []

    for tool in ["i2cget", "i2cset", "i2ctransfer"]:
        if shutil.which(tool) is None:
            missing.append(tool)

    if missing:
        raise RuntimeError(
            "缺少工具："
            + ", ".join(missing)
            + "\n请安装：sudo apt install -y i2c-tools"
        )


def i2cget_word(bus, addr, reg):
    output = run_cmd([
        "i2cget",
        "-y",
        str(bus),
        f"0x{addr:02x}",
        f"0x{reg:02x}",
        "w",
    ])

    return int(output, 16)


def i2cset_word(bus, addr, reg, value):
    run_cmd([
        "i2cset",
        "-y",
        str(bus),
        f"0x{addr:02x}",
        f"0x{reg:02x}",
        f"0x{value:04x}",
        "w",
    ])


def i2c_transfer_write(bus, addr, data):
    cmd = [
        "i2ctransfer",
        "-y",
        str(bus),
        f"w{len(data)}@0x{addr:02x}",
    ]

    cmd.extend([f"0x{x:02x}" for x in data])
    run_cmd(cmd)


def i2c_transfer_write_read(bus, addr, write_data, read_len):
    cmd = [
        "i2ctransfer",
        "-y",
        str(bus),
        f"w{len(write_data)}@0x{addr:02x}",
    ]

    cmd.extend([f"0x{x:02x}" for x in write_data])
    cmd.append(f"r{read_len}")

    output = run_cmd(cmd)

    values = []

    for part in output.split():
        values.append(int(part, 16))

    if len(values) != read_len:
        raise RuntimeError(f"读取长度错误，需要 {read_len} 字节，实际 {len(values)} 字节，输出={output}")

    return values


class VEML7700:
    def __init__(self, name, bus, addr=VEML7700_ADDR):
        self.name = name
        self.bus = int(bus)
        self.addr = int(addr)
        self.configured = False

    def configure(self):
        # 0x0000:
        # ALS power on
        # gain x1
        # integration time 100ms
        i2cset_word(self.bus, self.addr, VEML7700_REG_ALS_CONF, 0x0000)
        time.sleep(0.15)
        self.configured = True

    def read(self):
        if not self.configured:
            self.configure()

        als_raw = i2cget_word(self.bus, self.addr, VEML7700_REG_ALS)
        white_raw = i2cget_word(self.bus, self.addr, VEML7700_REG_WHITE)

        lux = als_raw * 0.0576

        return {
            "name": self.name,
            "bus": self.bus,
            "addr": self.addr,
            "als_raw": als_raw,
            "white_raw": white_raw,
            "lux": lux,
        }


class MS5837_02BA:
    def __init__(self, bus, addr):
        self.bus = int(bus)
        self.addr = int(addr)
        self.prom = None
        self.zero_pressure_hpa = None
        self.pressure_hpa = None
        self.temperature_c = None

    def reset(self):
        i2c_transfer_write(self.bus, self.addr, [MS5837_CMD_RESET])
        time.sleep(0.02)

    def read_prom_word(self, index):
        cmd = MS5837_CMD_PROM_READ_BASE + index * 2
        data = i2c_transfer_write_read(self.bus, self.addr, [cmd], 2)
        return (data[0] << 8) | data[1]

    def read_prom(self):
        prom = []

        for index in range(8):
            prom.append(self.read_prom_word(index))

        if not self.prom_is_valid(prom):
            raise RuntimeError(f"PROM 数据异常：{prom}")

        self.prom = prom
        return prom

    @staticmethod
    def prom_is_valid(prom):
        if prom is None or len(prom) != 8:
            return False

        if all(x == 0x0000 for x in prom):
            return False

        if all(x == 0xFFFF for x in prom):
            return False

        if all(x == 0 for x in prom[1:7]):
            return False

        return True

    def init(self):
        self.reset()
        self.read_prom()

    def read_adc(self):
        data = i2c_transfer_write_read(self.bus, self.addr, [MS5837_CMD_ADC_READ], 3)
        return (data[0] << 16) | (data[1] << 8) | data[2]

    def convert_and_read_adc(self, command):
        i2c_transfer_write(self.bus, self.addr, [command])
        time.sleep(0.02)
        return self.read_adc()

    def read_raw(self):
        d1 = self.convert_and_read_adc(MS5837_CMD_CONVERT_D1_8192)
        d2 = self.convert_and_read_adc(MS5837_CMD_CONVERT_D2_8192)

        return d1, d2

    def read(self):
        if self.prom is None:
            self.init()

        d1, d2 = self.read_raw()

        c = self.prom

        c1 = c[1]
        c2 = c[2]
        c3 = c[3]
        c4 = c[4]
        c5 = c[5]
        c6 = c[6]

        # MS5837-02BA 公式
        dT = d2 - c5 * 256.0
        temp = 2000.0 + dT * c6 / 8388608.0

        off = c2 * 131072.0 + c4 * dT / 64.0
        sens = c1 * 65536.0 + c3 * dT / 128.0

        ti = 0.0
        offi = 0.0
        sensi = 0.0

        if temp < 2000.0:
            temp_minus_2000 = temp - 2000.0
            ti = 11.0 * dT * dT / 34359738368.0
            offi = 31.0 * temp_minus_2000 * temp_minus_2000 / 8.0
            sensi = 63.0 * temp_minus_2000 * temp_minus_2000 / 32.0

        temp2 = temp - ti
        off2 = off - offi
        sens2 = sens - sensi

        pressure_001_mbar = (d1 * sens2 / 2097152.0 - off2) / 32768.0

        pressure_hpa = pressure_001_mbar / 100.0
        temperature_c = temp2 / 100.0

        self.pressure_hpa = pressure_hpa
        self.temperature_c = temperature_c

        depth_m = None

        if self.zero_pressure_hpa is not None:
            pressure_diff_pa = (pressure_hpa - self.zero_pressure_hpa) * 100.0
            depth_m = pressure_diff_pa / (WATER_DENSITY_KG_M3 * GRAVITY_M_S2)

        return {
            "bus": self.bus,
            "addr": self.addr,
            "pressure_hpa": pressure_hpa,
            "temperature_c": temperature_c,
            "depth_m": depth_m,
            "d1": d1,
            "d2": d2,
            "prom": self.prom,
        }

    def zero_depth(self):
        if self.pressure_hpa is not None:
            self.zero_pressure_hpa = self.pressure_hpa


def probe_ms5837(bus, addr):
    sensor = MS5837_02BA(bus, addr)

    try:
        sensor.init()
        return True, sensor.prom

    except Exception as error:
        return False, str(error)


def find_ms5837():
    buses = list_i2c_buses()

    for bus in buses:
        for addr in MS5837_ADDR_LIST:
            ok, detail = probe_ms5837(bus, addr)

            if ok:
                return bus, addr, detail

    return None, None, None


def print_header():
    print()
    print("=" * 100)
    print("RDK X5 I2C 传感器读取")
    print("Ctrl+C 退出")
    print("=" * 100)
    print()


def print_veml(data):
    print(
        f"{data['name']}: "
        f"/dev/i2c-{data['bus']} addr=0x{data['addr']:02X} | "
        f"ALS={data['als_raw']:6d} | "
        f"WHITE={data['white_raw']:6d} | "
        f"lux={data['lux']:10.3f}"
    )


def print_ms(data):
    depth_text = "未校零" if data["depth_m"] is None else f"{data['depth_m']:.4f} m"

    print(
        f"MS5837-02BA: "
        f"/dev/i2c-{data['bus']} addr=0x{data['addr']:02X} | "
        f"pressure={data['pressure_hpa']:10.3f} hPa | "
        f"temp={data['temperature_c']:8.3f} C | "
        f"depth={depth_text} | "
        f"D1={data['d1']} D2={data['d2']}"
    )


def main():
    parser = argparse.ArgumentParser(description="RDK X5 读取两个 VEML7700 和一个 MS5837-02BA")

    parser.add_argument("--veml1-bus", type=int, default=0)
    parser.add_argument("--veml2-bus", type=int, default=5)
    parser.add_argument("--veml-addr", type=str, default="0x10")

    parser.add_argument("--ms-bus", type=str, default="auto")
    parser.add_argument("--ms-addr", type=str, default="auto")

    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--zero-after", type=float, default=3.0, help="启动多少秒后自动给 MS5837 校零，默认 3 秒")

    args = parser.parse_args()

    check_tools()

    veml_addr = parse_int(args.veml_addr)

    veml1 = VEML7700("VEML7700-1", args.veml1_bus, veml_addr)
    veml2 = VEML7700("VEML7700-2", args.veml2_bus, veml_addr)

    if args.ms_bus == "auto" or args.ms_addr == "auto":
        print("正在自动扫描 MS5837，地址 0x76 / 0x77 ...")
        ms_bus, ms_addr, prom = find_ms5837()

        if ms_bus is None:
            print("没有找到 MS5837。")
            print("但程序会继续读取两个 VEML7700。")
            ms_sensor = None
        else:
            print(f"找到 MS5837：/dev/i2c-{ms_bus}, addr=0x{ms_addr:02X}, PROM={prom}")
            ms_sensor = MS5837_02BA(ms_bus, ms_addr)
            ms_sensor.init()
    else:
        ms_bus = parse_int(args.ms_bus)
        ms_addr = parse_int(args.ms_addr)
        ms_sensor = MS5837_02BA(ms_bus, ms_addr)
        ms_sensor.init()
        print(f"使用指定 MS5837：/dev/i2c-{ms_bus}, addr=0x{ms_addr:02X}, PROM={ms_sensor.prom}")

    print_header()

    start_time = time.time()
    zero_done = False

    try:
        while True:
            now_text = datetime.now().strftime("%H:%M:%S")
            print(f"[{now_text}]")

            try:
                data1 = veml1.read()
                print_veml(data1)
            except Exception as error:
                print(f"VEML7700-1: 读取失败：{error}")

            try:
                data2 = veml2.read()
                print_veml(data2)
            except Exception as error:
                print(f"VEML7700-2: 读取失败：{error}")

            if ms_sensor is not None:
                try:
                    ms_data = ms_sensor.read()

                    if not zero_done and time.time() - start_time >= args.zero_after:
                        ms_sensor.zero_depth()
                        zero_done = True
                        print(f"MS5837 已自动校零：zero_pressure={ms_sensor.zero_pressure_hpa:.3f} hPa")

                        ms_data = ms_sensor.read()

                    print_ms(ms_data)

                except Exception as error:
                    print(f"MS5837-02BA: 读取失败：{error}")
            else:
                print("MS5837-02BA: 未找到，未读取")

            print("-" * 100)
            time.sleep(args.interval)

    except KeyboardInterrupt:
        print()
        print("退出。")


if __name__ == "__main__":
    main()
