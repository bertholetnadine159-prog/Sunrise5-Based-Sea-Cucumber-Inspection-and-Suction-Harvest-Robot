#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import time

BUS = 1
ADDR = 0x76

CMD_RESET = 0x1E
CMD_ADC_READ = 0x00

# 先用 4096，稳定性比 8192 好一点
CMD_CONVERT_D1_4096 = 0x48
CMD_CONVERT_D2_4096 = 0x58

CMD_PROM_READ_BASE = 0xA0

WATER_DENSITY_KG_M3 = 997.0
GRAVITY_M_S2 = 9.80665


def run_cmd(cmd, retries=3, delay=0.03):
    last_error = None

    for _ in range(retries):
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
            )
            return result.stdout.strip()
        except subprocess.CalledProcessError as error:
            last_error = error
            time.sleep(delay)

    stderr = last_error.stderr.strip() if last_error and last_error.stderr else ""
    stdout = last_error.stdout.strip() if last_error and last_error.stdout else ""
    raise RuntimeError(
        f"命令失败：{' '.join(cmd)}\n"
        f"stdout={stdout}\n"
        f"stderr={stderr}"
    )


def i2c_write_byte(value):
    cmd = [
        "i2ctransfer",
        "-y",
        str(BUS),
        f"w1@0x{ADDR:02x}",
        f"0x{value:02x}",
    ]
    run_cmd(cmd)


def i2c_write_read(write_value, read_len):
    cmd = [
        "i2ctransfer",
        "-y",
        str(BUS),
        f"w1@0x{ADDR:02x}",
        f"0x{write_value:02x}",
        f"r{read_len}",
    ]

    output = run_cmd(cmd)
    values = [int(x, 16) for x in output.split()]

    if len(values) != read_len:
        raise RuntimeError(
            f"读取长度错误：需要 {read_len} 字节，实际 {len(values)} 字节，输出={output}"
        )

    return values


def reset_sensor():
    i2c_write_byte(CMD_RESET)
    time.sleep(0.08)


def read_prom_word(index):
    cmd = CMD_PROM_READ_BASE + index * 2
    data = i2c_write_read(cmd, 2)
    return (data[0] << 8) | data[1]


def read_prom():
    """
    MS5837 PROM:
        C0: factory/reserved
        C1~C6: 压力温度计算必须用
        C7: CRC

    你这里 0xAE，也就是 C7 读失败。
    C7 不参与压力计算，所以这里 C7 失败就跳过。
    """
    prom = [0] * 8

    print("读取 PROM：")

    for i in range(7):
        value = read_prom_word(i)
        prom[i] = value
        print(f"  C{i} = {value} / 0x{value:04X}")
        time.sleep(0.02)

    try:
        value = read_prom_word(7)
        prom[7] = value
        print(f"  C7 = {value} / 0x{value:04X}")
    except Exception as error:
        prom[7] = 0
        print("  C7 = 读取失败，先忽略 CRC，不影响压力温度计算")
        print(f"       {error}")

    if all(v == 0 for v in prom[1:7]):
        raise RuntimeError("C1~C6 全是 0，PROM 无效。")

    return prom


def read_adc():
    data = i2c_write_read(CMD_ADC_READ, 3)
    return (data[0] << 16) | (data[1] << 8) | data[2]


def convert_and_read_adc(command):
    i2c_write_byte(command)
    time.sleep(0.015)
    return read_adc()


def read_raw():
    d1 = convert_and_read_adc(CMD_CONVERT_D1_4096)
    d2 = convert_and_read_adc(CMD_CONVERT_D2_4096)
    return d1, d2


def calculate_ms5837_02ba(prom, d1, d2):
    c1 = prom[1]
    c2 = prom[2]
    c3 = prom[3]
    c4 = prom[4]
    c5 = prom[5]
    c6 = prom[6]

    dT = d2 - c5 * 256.0

    temp = 2000.0 + dT * c6 / 8388608.0

    # MS5837-02BA
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

    return pressure_hpa, temperature_c


def main():
    print(f"MS5837-02BA 读取程序：/dev/i2c-{BUS}, addr=0x{ADDR:02X}")
    print("按 Ctrl+C 退出")
    print()

    reset_sensor()
    prom = read_prom()

    print()
    print("开始读取，前 3 秒自动作为零点压力")
    print()

    zero_pressure = None
    zero_samples = []
    start_time = time.time()

    try:
        while True:
            d1, d2 = read_raw()
            pressure_hpa, temperature_c = calculate_ms5837_02ba(prom, d1, d2)

            if time.time() - start_time < 3.0:
                zero_samples.append(pressure_hpa)
                zero_pressure = sum(zero_samples) / len(zero_samples)

            if zero_pressure is None:
                depth_m = 0.0
            else:
                pressure_diff_pa = (pressure_hpa - zero_pressure) * 100.0
                depth_m = pressure_diff_pa / (WATER_DENSITY_KG_M3 * GRAVITY_M_S2)

            print(
                f"pressure={pressure_hpa:10.3f} hPa, "
                f"temp={temperature_c:8.3f} °C, "
                f"depth={depth_m:8.4f} m, "
                f"zero={zero_pressure if zero_pressure else 0:10.3f} hPa, "
                f"D1={d1}, D2={d2}"
            )

            time.sleep(0.5)

    except KeyboardInterrupt:
        print()
        print("退出。")


if __name__ == "__main__":
    main()
