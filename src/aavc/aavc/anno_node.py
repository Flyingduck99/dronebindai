#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSinkNode(Node):
    def __init__(self):
        super().__init__('image_sink_node')
        self.bridge = CvBridge()

        # ปรับได้ตามต้องการ
        self.save_dir = self.declare_parameter('save_dir', 'annotated_captures').get_parameter_value().string_value
        self.show_window = self.declare_parameter('show_window', False).get_parameter_value().bool_value  # True ถ้าจะเปิด window
        self.save_every_n = self.declare_parameter('save_every_n', 1).get_parameter_value().integer_value  # เซฟทุก N เฟรม

        os.makedirs(self.save_dir, exist_ok=True)
        self.frame_count = 0

        # Subscribe ภาพ annotate จาก YOLO
        self.sub = self.create_subscription(
            Image,
            '/camera/image_anno',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f"ImageSinkNode started. Subscribing /yolo/image_annotated | save_dir='{self.save_dir}' | "
            f"save_every_n={self.save_every_n} | show_window={self.show_window}"
        )

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge conversion error: {e}')
            return

        self.frame_count += 1

        # บันทึกรูปทุก N เฟรม
        if self.frame_count % self.save_every_n == 0:
            stamp_ns = self.get_clock().now().nanoseconds
            filename = os.path.join(self.save_dir, f'annot_{stamp_ns}.jpg')
            try:
                cv2.imwrite(filename, cv_img)
                self.get_logger().info(f'Saved: {filename}')
            except Exception as e:
                self.get_logger().error(f'Failed to save image: {e}')

        # เปิดหน้าต่างดูภาพ (ถ้ามีจอ)
        if self.show_window:
            cv2.imshow('YOLO Annotated', cv_img)
            # ต้องมี waitKey เล็กน้อยเพื่ออัพเดทหน้าต่าง
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

