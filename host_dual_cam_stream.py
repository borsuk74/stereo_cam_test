#!/usr/bin/env python3
"""
Runs on the HOST (not in the container). Captures both CSI cameras via
nvarguscamerasrc/GStreamer (the pipeline the user confirmed works) and
streams JPEG frames over TCP, one port per camera, so a ROS node inside
the container (run with --network host) can pick them up on 127.0.0.1.

Wire format per frame, sent to each connected client:
  4 bytes  little-endian uint32  length of JPEG payload
  N bytes  JPEG payload
"""
import socket
import struct
import threading
import time

import cv2

CAM_LEFT = 0
CAM_RIGHT = 1

PORTS = {CAM_LEFT: 6000, CAM_RIGHT: 6001}


def gstreamer_pipeline(sensor_id, capture_width=1920, capture_height=1080, framerate=30):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv ! video/x-raw, format=(string)BGRx ! "
        f"videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    )


class FrameServer:
    """Accepts one or more TCP clients and fans out the latest JPEG frame to each."""

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

    def send(self, jpeg_bytes):
        header = struct.pack("<I", len(jpeg_bytes))
        with self.lock:
            dead = []
            for conn in self.clients:
                try:
                    conn.sendall(header)
                    conn.sendall(jpeg_bytes)
                except OSError:
                    dead.append(conn)
            for conn in dead:
                self.clients.remove(conn)
                try:
                    conn.close()
                except OSError:
                    pass


def main():
    cap_left = cv2.VideoCapture(gstreamer_pipeline(CAM_LEFT), cv2.CAP_GSTREAMER)
    cap_right = cv2.VideoCapture(gstreamer_pipeline(CAM_RIGHT), cv2.CAP_GSTREAMER)
    if not cap_left.isOpened() or not cap_right.isOpened():
        raise RuntimeError("Could not open both cameras")

    server_left = FrameServer(PORTS[CAM_LEFT])
    server_right = FrameServer(PORTS[CAM_RIGHT])
    print(f"Serving cam0 on tcp:{PORTS[CAM_LEFT]}, cam1 on tcp:{PORTS[CAM_RIGHT]}")

    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 90]

    try:
        while True:
            ok_left, frame_left = cap_left.read()
            ok_right, frame_right = cap_right.read()
            if not ok_left or not ok_right:
                print("frame read failed, stopping")
                break

            ok, buf = cv2.imencode(".jpg", frame_left, encode_params)
            if ok:
                server_left.send(buf.tobytes())

            ok, buf = cv2.imencode(".jpg", frame_right, encode_params)
            if ok:
                server_right.send(buf.tobytes())
    except KeyboardInterrupt:
        pass
    finally:
        cap_left.release()
        cap_right.release()


if __name__ == "__main__":
    main()
