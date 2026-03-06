#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String

class ColorClassificationNode(Node):
    def __init__(self):
        super().__init__('color_classification_node')

        # Subscribe YOLO detections
        self.create_subscription(String, "/yolo/detections", self.yolo_callback, 10)

        # Publish color detections
        self.pub = self.create_publisher(String, "/color/detections", 10)

        self.get_logger().info("ColorClassificationNode started (label → color passthrough).")

    def yolo_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"[color_node] JSON decode error: {e}")
            return

        detections = data.get("detections", [])
        if not detections:
            return

        # Copy label → color
        for det in detections:
            det["color"] = det.get("label", "undefined")

        # ส่งต่อในรูปแบบเดิม
        out = {"detections": detections}
        msg_out = String()
        msg_out.data = json.dumps(out)
        self.pub.publish(msg_out)

        self.get_logger().debug("Published color detections.")

def main(args=None):
    rclpy.init(args=args)
    node = ColorClassificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

