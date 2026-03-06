#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class LowLatencyViewer(Node):
    def __init__(self):
        super().__init__('low_latency_viewer')
        # Subscribe topic /out/compressed
        self.sub = self.create_subscription(
            CompressedImage,
            '/out/compressed',
            self.callback,
            10
        )
        self.get_logger().info("Subscribed to /out/compressed, ready to show frames!")

    def callback(self, msg: CompressedImage):
        # Convert ROS CompressedImage to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is not None:
            # Show frame in window
            cv2.imshow("Low-Latency Viewer", frame)
            # Wait 1 ms to allow GUI update
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LowLatencyViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
