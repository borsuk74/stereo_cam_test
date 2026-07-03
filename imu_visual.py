#!/usr/bin/env python3
"""Real-time 3D visualization of ICM-20948 IMU orientation.

Reads accel/gyro/mag over I2C, fuses them into an orientation quaternion
with a Mahony AHRS filter (ported from icm20948-demo/ICM20948.c), and
draws a live-rotating gizmo + roll/pitch/yaw readout.

Usage:
    python3 imu_visual.py --bus 7      # read a real ICM-20948 on /dev/i2c-7
    python3 imu_visual.py --demo       # no hardware needed, spins a fake IMU

Requires: numpy, matplotlib, and (for real hardware) smbus2 (`pip install smbus2`).
"""
import argparse
import math
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  registers the '3d' projection

# ---------------------------------------------------------------------------
# ICM-20948 register map (mirrors icm20948-demo/ICM20948.h)
# ---------------------------------------------------------------------------
I2C_ADDR_ICM20948 = 0x68
I2C_ADDR_AK09916 = 0x0C
AK09916_READ_FLAG = 0x80
AK09916_WRITE_FLAG = 0x00

REG_WIA = 0x00
WIA_VAL = 0xEA
REG_USER_CTRL = 0x03
BIT_I2C_MST_EN = 0x20
REG_PWR_MGMT_1 = 0x06
VAL_ALL_RESET = 0x80
VAL_RUN_MODE = 0x01
REG_BANK_SEL = 0x7F
BANK_0, BANK_2, BANK_3 = 0x00, 0x20, 0x30

ACCEL_REGS = [(0x2D, 0x2E), (0x2F, 0x30), (0x31, 0x32)]  # (H, L) per axis X,Y,Z
GYRO_REGS = [(0x33, 0x34), (0x35, 0x36), (0x37, 0x38)]
REG_EXT_SENS_DATA_00 = 0x3B

REG_GYRO_SMPLRT_DIV = 0x00
REG_GYRO_CONFIG_1 = 0x01
GYRO_DLPCFG_6 = 0x30
GYRO_FS_1000DPS = 0x04
GYRO_DLPF_EN = 0x01
REG_ACCEL_SMPLRT_DIV_2 = 0x11
REG_ACCEL_CONFIG = 0x14
ACCEL_DLPCFG_6 = 0x30
ACCEL_FS_2G = 0x00
ACCEL_DLPF_EN = 0x01

REG_I2C_SLV0_ADDR = 0x03
REG_I2C_SLV0_REG = 0x04
REG_I2C_SLV0_CTRL = 0x05
BIT_SLV0_EN = 0x80
REG_I2C_SLV1_ADDR = 0x07
REG_I2C_SLV1_REG = 0x08
REG_I2C_SLV1_CTRL = 0x09
REG_I2C_SLV1_DO = 0x0A

MAG_REG_WIA1, MAG_WIA1_VAL = 0x00, 0x48
MAG_REG_WIA2, MAG_WIA2_VAL = 0x01, 0x09
MAG_REG_ST2 = 0x10
MAG_REG_DATA = 0x11
MAG_REG_CNTL2 = 0x31
MAG_MODE_20HZ = 0x04
MAG_DATA_LEN = 6

GYRO_SSF_1000DPS = 32.8   # LSB/dps
ACCEL_SSF_2G = 16384.0    # LSB/g
MAG_SSF_UT = 0.15         # uT/LSB


class RollingAvg:
    """8-sample moving average, matching icm20948CalAvgValue() in the C driver."""

    def __init__(self):
        self._buf = deque([0] * 8, maxlen=8)

    def add(self, value):
        self._buf.append(value)
        return sum(self._buf) // 8


class ICM20948:
    """I2C driver for the ICM-20948 + AK09916, ported register-for-register
    from icm20948-demo/ICM20948.c so it behaves like the proven C demo."""

    def __init__(self, bus_num):
        import smbus2
        self.bus = smbus2.SMBus(bus_num)
        self._gyro_offset = (0, 0, 0)
        self._gyro_avg = [RollingAvg() for _ in range(3)]
        self._accel_avg = [RollingAvg() for _ in range(3)]
        self._mag_avg = [RollingAvg() for _ in range(3)]

    def _rd(self, dev_addr, reg):
        return self.bus.read_byte_data(dev_addr, reg)

    def _wr(self, dev_addr, reg, val):
        self.bus.write_byte_data(dev_addr, reg, val)

    def _bank(self, bank):
        self._wr(I2C_ADDR_ICM20948, REG_BANK_SEL, bank)

    def _read_secondary(self, i2c_addr, reg_addr, length):
        self._bank(BANK_3)
        self._wr(I2C_ADDR_ICM20948, REG_I2C_SLV0_ADDR, i2c_addr)
        self._wr(I2C_ADDR_ICM20948, REG_I2C_SLV0_REG, reg_addr)
        self._wr(I2C_ADDR_ICM20948, REG_I2C_SLV0_CTRL, BIT_SLV0_EN | length)

        self._bank(BANK_0)
        ctrl = self._rd(I2C_ADDR_ICM20948, REG_USER_CTRL)
        self._wr(I2C_ADDR_ICM20948, REG_USER_CTRL, ctrl | BIT_I2C_MST_EN)
        time.sleep(0.005)
        self._wr(I2C_ADDR_ICM20948, REG_USER_CTRL, ctrl & ~BIT_I2C_MST_EN)

        data = [self._rd(I2C_ADDR_ICM20948, REG_EXT_SENS_DATA_00 + i) for i in range(length)]

        self._bank(BANK_3)
        ctrl = self._rd(I2C_ADDR_ICM20948, REG_I2C_SLV0_CTRL)
        self._wr(I2C_ADDR_ICM20948, REG_I2C_SLV0_CTRL, ctrl & ~BIT_SLV0_EN)
        self._bank(BANK_0)
        return data

    def _write_secondary(self, i2c_addr, reg_addr, value):
        self._bank(BANK_3)
        self._wr(I2C_ADDR_ICM20948, REG_I2C_SLV1_ADDR, i2c_addr)
        self._wr(I2C_ADDR_ICM20948, REG_I2C_SLV1_REG, reg_addr)
        self._wr(I2C_ADDR_ICM20948, REG_I2C_SLV1_DO, value)
        self._wr(I2C_ADDR_ICM20948, REG_I2C_SLV1_CTRL, BIT_SLV0_EN | 1)

        self._bank(BANK_0)
        ctrl = self._rd(I2C_ADDR_ICM20948, REG_USER_CTRL)
        self._wr(I2C_ADDR_ICM20948, REG_USER_CTRL, ctrl | BIT_I2C_MST_EN)
        time.sleep(0.005)
        self._wr(I2C_ADDR_ICM20948, REG_USER_CTRL, ctrl & ~BIT_I2C_MST_EN)

        self._bank(BANK_3)
        ctrl = self._rd(I2C_ADDR_ICM20948, REG_I2C_SLV0_CTRL)
        self._wr(I2C_ADDR_ICM20948, REG_I2C_SLV0_CTRL, ctrl & ~BIT_SLV0_EN)
        self._bank(BANK_0)

    def check(self):
        return self._rd(I2C_ADDR_ICM20948, REG_WIA) == WIA_VAL

    def init(self):
        if not self.check():
            raise RuntimeError("ICM-20948 not found on this bus (WHO_AM_I mismatch)")

        self._bank(BANK_0)
        self._wr(I2C_ADDR_ICM20948, REG_PWR_MGMT_1, VAL_ALL_RESET)
        time.sleep(0.01)
        self._wr(I2C_ADDR_ICM20948, REG_PWR_MGMT_1, VAL_RUN_MODE)

        self._bank(BANK_2)
        self._wr(I2C_ADDR_ICM20948, REG_GYRO_SMPLRT_DIV, 0x07)
        self._wr(I2C_ADDR_ICM20948, REG_GYRO_CONFIG_1,
                 GYRO_DLPCFG_6 | GYRO_FS_1000DPS | GYRO_DLPF_EN)
        self._wr(I2C_ADDR_ICM20948, REG_ACCEL_SMPLRT_DIV_2, 0x07)
        self._wr(I2C_ADDR_ICM20948, REG_ACCEL_CONFIG,
                 ACCEL_DLPCFG_6 | ACCEL_FS_2G | ACCEL_DLPF_EN)

        self._bank(BANK_0)
        time.sleep(0.1)

        self._calibrate_gyro()

        wia1, wia2 = self._read_secondary(I2C_ADDR_AK09916 | AK09916_READ_FLAG, MAG_REG_WIA1, 2)
        if wia1 != MAG_WIA1_VAL or wia2 != MAG_WIA2_VAL:
            raise RuntimeError("AK09916 magnetometer not found behind the ICM-20948")
        self._write_secondary(I2C_ADDR_AK09916 | AK09916_WRITE_FLAG, MAG_REG_CNTL2, MAG_MODE_20HZ)

    def _read_axes(self, reg_pairs, averagers):
        out = []
        for (reg_h, reg_l), avg in zip(reg_pairs, averagers):
            lo = self._rd(I2C_ADDR_ICM20948, reg_l)
            hi = self._rd(I2C_ADDR_ICM20948, reg_h)
            val = (hi << 8) | lo
            if val >= 0x8000:
                val -= 0x10000
            out.append(avg.add(val))
        return out

    def _calibrate_gyro(self):
        sums = [0, 0, 0]
        for _ in range(32):
            raw = self._read_axes(GYRO_REGS, self._gyro_avg)
            for i in range(3):
                sums[i] += raw[i]
            time.sleep(0.01)
        self._gyro_offset = tuple(s >> 5 for s in sums)

    def read_accel_raw(self):
        return self._read_axes(ACCEL_REGS, self._accel_avg)

    def read_gyro_raw(self):
        raw = self._read_axes(GYRO_REGS, self._gyro_avg)
        return [raw[i] - self._gyro_offset[i] for i in range(3)]

    def read_mag_raw(self):
        for _ in range(20):
            status = self._read_secondary(I2C_ADDR_AK09916 | AK09916_READ_FLAG, MAG_REG_ST2, 1)[0]
            if status & 0x01:
                break
            time.sleep(0.01)
        else:
            raw = [0, 0, 0]
            return [avg.add(v) for avg, v in zip(self._mag_avg, raw)]

        data = self._read_secondary(I2C_ADDR_AK09916 | AK09916_READ_FLAG, MAG_REG_DATA, MAG_DATA_LEN)
        raw = []
        for i in range(3):
            val = (data[2 * i + 1] << 8) | data[2 * i]
            if val >= 0x8000:
                val -= 0x10000
            raw.append(val)
        raw = [raw[0], -raw[1], -raw[2]]  # matches icm20948MagRead()'s sign flip
        return [avg.add(v) for avg, v in zip(self._mag_avg, raw)]


class MahonyAHRS:
    """Quaternion complementary filter, ported line-for-line from imuAHRSupdate()
    in ICM20948.c (including its quirk of chaining each new q_i into the next
    update within the same step) so it matches the tuned/proven behavior."""

    KP = 4.50
    KI = 1.00
    HALF_T = 0.024  # fixed step matching the original ~50 Hz sample loop

    def __init__(self):
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        norm = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
        ax, ay, az = ax * norm, ay * norm, az * norm
        norm = 1.0 / math.sqrt(mx * mx + my * my + mz * mz)
        mx, my, mz = mx * norm, my * norm, mz * norm

        q0q0, q0q1, q0q2, q0q3 = q0 * q0, q0 * q1, q0 * q2, q0 * q3
        q1q1, q1q2, q1q3 = q1 * q1, q1 * q2, q1 * q3
        q2q2, q2q3 = q2 * q2, q2 * q3
        q3q3 = q3 * q3

        hx = 2 * mx * (0.5 - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2)
        hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5 - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1)
        hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5 - q1q1 - q2q2)
        bx = math.sqrt(hx * hx + hy * hy)
        bz = hz

        vx = 2 * (q1q3 - q0q2)
        vy = 2 * (q0q1 + q2q3)
        vz = q0q0 - q1q1 - q2q2 + q3q3
        wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2)
        wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3)
        wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2)

        ex = (ay * vz - az * vy) + (my * wz - mz * wy)
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz)
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx)

        if ex != 0.0 and ey != 0.0 and ez != 0.0:
            gx += self.KP * ex + self.KI * ex * self.HALF_T
            gy += self.KP * ey + self.KI * ey * self.HALF_T
            gz += self.KP * ez + self.KI * ez * self.HALF_T

        n0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * self.HALF_T
        n1 = q1 + (n0 * gx + q2 * gz - q3 * gy) * self.HALF_T
        n2 = q2 + (n0 * gy - n1 * gz + q3 * gx) * self.HALF_T
        n3 = q3 + (n0 * gz + n1 * gy - n2 * gx) * self.HALF_T

        norm = 1.0 / math.sqrt(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3)
        self.q0, self.q1, self.q2, self.q3 = n0 * norm, n1 * norm, n2 * norm, n3 * norm

    def angles_deg(self):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -2 * q1 * q3 + 2 * q0 * q2))))
        roll = math.degrees(math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1))
        yaw = math.degrees(math.atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1))
        return roll, pitch, yaw

    def rotation_matrix(self):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        return np.array([
            [1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)],
        ])


class IMU:
    """Couples the sensor driver + filter, mirroring imuDataGet() in ICM20948.c."""

    def __init__(self, bus_num):
        self.sensor = ICM20948(bus_num)
        self.ahrs = MahonyAHRS()

    def init(self):
        self.sensor.init()

    def read(self):
        accel_raw = self.sensor.read_accel_raw()
        gyro_raw = self.sensor.read_gyro_raw()
        mag_raw = self.sensor.read_mag_raw()

        gx = math.radians(gyro_raw[0] / GYRO_SSF_1000DPS)
        gy = math.radians(gyro_raw[1] / GYRO_SSF_1000DPS)
        gz = math.radians(gyro_raw[2] / GYRO_SSF_1000DPS)
        self.ahrs.update(gx, gy, gz, *accel_raw, *mag_raw)

        roll, pitch, yaw = self.ahrs.angles_deg()
        accel_g = tuple(v / ACCEL_SSF_2G for v in accel_raw)
        gyro_dps = tuple(v / GYRO_SSF_1000DPS for v in gyro_raw)
        mag_ut = tuple(v * MAG_SSF_UT for v in mag_raw)
        return roll, pitch, yaw, accel_g, gyro_dps, mag_ut


class DemoIMU:
    """Fake IMU for exercising the visualizer without hardware: feeds a slowly
    tumbling, physically-consistent accel/mag pair through the real AHRS filter."""

    def __init__(self):
        self.ahrs = MahonyAHRS()
        self.t0 = time.time()

    def init(self):
        pass

    def read(self):
        t = time.time() - self.t0
        gx, gy, gz = math.radians(12), math.radians(7), math.radians(-5)
        # gravity as seen by a body rotating at (gx, gy, gz) rad/s
        roll_sim, pitch_sim = 0.6 * math.sin(0.3 * t), 0.5 * math.sin(0.2 * t)
        ax = -math.sin(pitch_sim)
        ay = math.sin(roll_sim) * math.cos(pitch_sim)
        az = math.cos(roll_sim) * math.cos(pitch_sim)
        mx, my, mz = math.cos(0.1 * t), math.sin(0.1 * t), 0.3
        self.ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        roll, pitch, yaw = self.ahrs.angles_deg()
        return roll, pitch, yaw, (ax, ay, az), (12.0, 7.0, -5.0), (mx * 50, my * 50, mz * 50)


def make_gizmo():
    hx, hy, hz = 1.0, 0.5, 0.15
    box = np.array([
        [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
        [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
    ])
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
              (0, 4), (1, 5), (2, 6), (3, 7)]
    axes = np.array([[1.6, 0, 0], [0, 1.6, 0], [0, 0, 1.6]])
    return box, edges, axes


def run(imu, interval_ms):
    box, edges, axes = make_gizmo()
    axis_colors = ("r", "g", "b")
    axis_labels = ("X", "Y", "Z")

    fig = plt.figure("IMU orientation", figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")

    def draw(_frame):
        try:
            roll, pitch, yaw, accel_g, gyro_dps, mag_ut = imu.read()
        except OSError as exc:
            print(f"IMU read failed: {exc}")
            return []

        r = imu.ahrs.rotation_matrix()
        world_box = box @ r.T
        world_axes = axes @ r.T

        ax.cla()
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_zlim(-2, 2)
        ax.set_box_aspect((1, 1, 1))
        ax.set_title("ICM-20948 orientation (live)")

        for i0, i1 in edges:
            p0, p1 = world_box[i0], world_box[i1]
            ax.plot3D(*zip(p0, p1), color="dimgray")

        for vec, color, label in zip(world_axes, axis_colors, axis_labels):
            ax.plot3D([0, vec[0]], [0, vec[1]], [0, vec[2]], color=color, linewidth=2)
            ax.text(*vec, label, color=color)

        ax.text2D(
            0.02, 0.98,
            f"Roll:  {roll:6.1f}\N{DEGREE SIGN}\n"
            f"Pitch: {pitch:6.1f}\N{DEGREE SIGN}\n"
            f"Yaw:   {yaw:6.1f}\N{DEGREE SIGN}\n\n"
            f"Accel(g): {accel_g[0]:.2f} {accel_g[1]:.2f} {accel_g[2]:.2f}\n"
            f"Gyro(dps): {gyro_dps[0]:.1f} {gyro_dps[1]:.1f} {gyro_dps[2]:.1f}\n"
            f"Mag(uT): {mag_ut[0]:.1f} {mag_ut[1]:.1f} {mag_ut[2]:.1f}",
            transform=ax.transAxes, va="top", family="monospace", fontsize=9,
        )
        return []

    anim = FuncAnimation(fig, draw, interval=interval_ms, cache_frame_data=False)
    plt.show()
    return anim


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=7, help="I2C bus number, e.g. 7 for /dev/i2c-7")
    parser.add_argument("--interval", type=int, default=50, help="plot update interval in ms")
    parser.add_argument("--demo", action="store_true", help="run without hardware using simulated data")
    args = parser.parse_args()

    if args.demo:
        imu = DemoIMU()
    else:
        imu = IMU(args.bus)

    imu.init()
    if not args.demo:
        print("ICM-20948 initialized, starting visualization (close the plot window to quit)...")
    run(imu, args.interval)


if __name__ == "__main__":
    main()
