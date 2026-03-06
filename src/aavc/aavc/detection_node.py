#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import os
import cv2
import numpy as np
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf_transformations import euler_from_quaternion
import time



class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        self.bridge = CvBridge()

        # Sensor state — updated by individual callbacks
        self.current_gps   = None
        self.current_alt   = None
        self.current_roll  = 0.0
        self.current_pitch = 0.0
        self.current_yaw   = 0.0

        # Frame buffer — stores both the image and its ROS header stamp
        self.latest_frame       = None
        self.latest_frame_stamp = None  # builtin_interfaces/Time from camera header

        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(Image,      "/camera/image",                        self.image_callback, image_qos)
        self.create_subscription(NavSatFix,  "/mavros/global_position/global",       self.gps_callback,   qos_profile_sensor_data)
        self.create_subscription(Float64,    "/mavros/global_position/rel_alt",      self.alt_callback,   qos_profile_sensor_data)
        self.create_subscription(PoseStamped,"/mavros/local_position/pose",          self.pose_callback,  qos_profile_sensor_data)

        # Publishers
        self.pub_detection = self.create_publisher(String, "/yolo/detections", 10)
        self.pub_cam = self.create_publisher(Image, "/camera/image_anno", 10)

        # Load YOLO model
        self.model = YOLO(r"/home/ciimav/ros2_ws/src/aavc/aavc/weight/bestforfinalproject.pt")

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.process_frame)

        self.get_logger().info("✅ YOLO Detection Node started.")

    def image_callback(self, msg):
        try:
            self.latest_frame       = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_frame_stamp = msg.header.stamp  # preserve capture timestamp
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)

    def alt_callback(self, msg):
        self.current_alt = float(msg.data)

    def pose_callback(self, msg):
        q = msg.pose.orientation
        r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_roll, self.current_pitch, self.current_yaw = r, p, y

    def process_frame(self):
        if self.latest_frame is None or self.current_gps is None:
            return
        start_time = time.time()

        # --- Atomic sensor snapshot at frame-processing time ---
        # All fields are read together so geolocation uses a consistent state.
        frame       = self.latest_frame
        frame_stamp = self.latest_frame_stamp
        snap_gps    = self.current_gps
        snap_alt    = self.current_alt
        snap_roll   = self.current_roll
        snap_pitch  = self.current_pitch
        snap_yaw    = self.current_yaw
        self.latest_frame       = None
        self.latest_frame_stamp = None

        if frame is None:
            return
        frame = frame.copy()
        results = self.model.predict(frame,imgsz = 480,conf=0.5)
        detection_list = []
        annotated_frame = frame.copy()

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls)
                conf = float(box.conf)
                label = self.model.names[class_id]

                if conf < 0.3:
                    continue

                X1, Y1, X2, Y2 = map(int, box.xyxy[0])
                cx = int((X1 + X2) / 2)
                cy = int((Y1 + Y2) / 2)

                
                cv2.rectangle(annotated_frame, (X1, Y1), (X2, Y2), (0, 255, 0), 3)
                cv2.putText(annotated_frame, f"{label} {conf:.2f}", (X1, Y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                detection_list.append({
                    "label": label,
                    "confidence": round(conf, 2),
                    "bbox": [X1, Y1, X2, Y2],
                    "pixel_center": [cx, cy],
                    # Full sensor snapshot taken at the moment this frame was processed.
                    # geolocate_node uses these instead of its own (potentially stale) subscriptions.
                    "uav_gps":   [round(snap_gps[0], 7), round(snap_gps[1], 7)],
                    "uav_alt":   round(snap_alt, 3) if snap_alt is not None else None,
                    "uav_roll":  round(snap_roll,  6),
                    "uav_pitch": round(snap_pitch, 6),
                    "uav_yaw":   round(snap_yaw,   6),
                    # Frame timestamp lets cluster_node group detections by actual capture time.
                    "frame_stamp": {
                        "sec":     frame_stamp.sec     if frame_stamp else 0,
                        "nanosec": frame_stamp.nanosec if frame_stamp else 0,
                    },
                })
                
                fps = 1 / (time.time() - start_time + 1e-6) 
                cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if detection_list:
            
            cv2.putText(annotated_frame,
                        f"GPS: {snap_gps[0]:.6f}, {snap_gps[1]:.6f}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

            
            save_dir = "/home/ciimav/ros2_ws/annotated_captures"
            os.makedirs(save_dir, exist_ok=True)
            filename = f"{save_dir}/capture_{self.get_clock().now().nanoseconds}.jpg"
            cv2.imwrite(filename, annotated_frame)

            
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.pub_cam.publish(annotated_msg)

            
            msg = String()
            msg.data = json.dumps({"detections": detection_list})
            self.pub_detection.publish(msg)

            self.get_logger().info(f"📤 Published {len(detection_list)} detections -> {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
