#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import time

BUS = 0
ADDR = 0x10

REG_ALS_CONF = 0x00
REG_ALS = 0x04
REG_WHITE = 0x05


def run_cmd(cmd):
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def i2cset_word(bus, addr, reg, value):
    cmd = [
        "i2cset",
        "-y",
        str(bus),
        f"0x{addr:02x}",
        f"0x{reg:02x}",
        f"0x{value:04x}",
        "w",
    ]
    run_cmd(cmd)


def i2cget_word(bus, addr, reg):
    cmd = [
        "i2cget",
        "-y",
        str(bus),
        f"0x{addr:02x}",
        f"0x{reg:02x}",
        "w",
    ]

    output = run_cmd(cmd)
    return int(output, 16)


def main():
    print("VEML7700 当前光照读取")
    print(f"I2C bus=/dev/i2c-{BUS}, addr=0x{ADDR:02X}")
    print("按 Ctrl+C 退出")
    print()

    # 配置 VEML7700：
    # 0x0000 = ALS power on, gain x1, integration time 100ms
    i2cset_word(BUS, ADDR, REG_ALS_CONF, 0x0000)

    time.sleep(0.5)

    try:
        while True:
            als_raw = i2cget_word(BUS, ADDR, REG_ALS)
            white_raw = i2cget_word(BUS, ADDR, REG_WHITE)

            # gain x1 + integration time 100ms 时，约 0.0576 lux/count
            lux = als_raw * 0.0576

            print(
                f"ALS原始值={als_raw:6d}, "
                f"ALS十六进制=0x{als_raw:04X}, "
                f"WHITE原始值={white_raw:6d}, "
                f"光照={lux:9.3f} lux"
            )

            time.sleep(0.5)

    except KeyboardInterrupt:
        print()
        print("退出。")


if __name__ == "__main__":
    main()
