#!/usr/bin/env python3
"""
Runs on the HOST (not in the container), in the same directory as
imu_visual.py (reuses its proven ICM-20948 driver + Mahony AHRS filter).
Streams IMU samples over TCP so a ROS node inside the container (run with
--network host, or reachable via the docker bridge at 172.17.0.1) can pick
them up, mirroring the approach used for the cameras in
host_dual_cam_stream.py.

Wire format per sample, sent to each connected client - fixed 88-byte
little-endian struct of 11 doubles:
  t, qx, qy, qz, qw, gx, gy, gz, ax, ay, az

  t                 host timestamp (time.time())
  qx, qy, qz, qw    fused orientation quaternion (Mahony AHRS)
  gx, gy, gz        angular velocity, rad/s
  ax, ay, az        linear acceleration, m/s^2
"""
import math
import socket
import struct
import threading
import time

from imu_visual import IMU

PORT = 6002
RATE_HZ = 50
G = 9.80665

STRUCT_FMT = "<11d"


class FrameServer:
    """Accepts one or more TCP clients and fans out each sample to all of them."""

    def __init__(self, port):
        self.port = port
        self.clients = []
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.listen(4)
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        while True:
            conn, addr = self.sock.accept()
            print(f"[port {self.port}] client connected: {addr}")
            with self.lock:
                self.clients.append(conn)

    def send(self, payload):
        with self.lock:
            dead = []
            for conn in self.clients:
                try:
                    conn.sendall(payload)
                except OSError:
                    dead.append(conn)
            for conn in dead:
                self.clients.remove(conn)
                try:
                    conn.close()
                except OSError:
                    pass


def main():
    imu = IMU(bus_num=7)
    imu.init()
    print("ICM-20948 initialized")

    server = FrameServer(PORT)
    print(f"Serving imu on tcp:{PORT}")

    period = 1.0 / RATE_HZ
    try:
        while True:
            t0 = time.time()

            roll, pitch, yaw, accel_g, gyro_dps, mag_ut = imu.read()

            # imu.ahrs.q0..q3 are (w, x, y, z)
            qw, qx, qy, qz = imu.ahrs.q0, imu.ahrs.q1, imu.ahrs.q2, imu.ahrs.q3

            gx, gy, gz = (math.radians(v) for v in gyro_dps)
            ax, ay, az = (v * G for v in accel_g)

            payload = struct.pack(STRUCT_FMT, time.time(), qx, qy, qz, qw, gx, gy, gz, ax, ay, az)
            server.send(payload)

            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
