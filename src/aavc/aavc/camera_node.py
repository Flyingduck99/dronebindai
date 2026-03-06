#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2
import threading
import time
import numpy as np


class FastCameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.camera_id = 0
        self.width = 640
        self.height = 480
        self.fps = 10   # ??????????? YOLO (~4–10 FPS)

        # QoS ?????? video
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publisher
        self.pub = self.create_publisher(
            Image,
            "/camera/image",
            qos_profile,
        )

        # Open camera
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            return

        # ??? raw YUYV/BGR ????????? MJPG
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

        self.get_logger().info("raw camera node started")

    def loop(self):
        frame_interval = 1.0 / self.fps
        last_time = time.perf_counter()

        while self.running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                continue

            # OpenCV ??? BGR ????????
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()   # ? copy ????????? ???????

            self.pub.publish(msg)

            el = time.perf_counter() - last_time
            if el < frame_interval:
                time.sleep(frame_interval - el)
            last_time = time.perf_counter()

    def destroy_node(self):
        self.running = False
        time.sleep(0.1)
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = FastCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
