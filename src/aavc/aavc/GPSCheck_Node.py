#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
import threading
from pyproj import Transformer
import numpy as np

class GPSCheck(Node):
    def __init__(self):
        super().__init__("GPSCheck_node")
        
        #parameter
        self.declare_parameter("gps_topic", "/mavros/global_position/global")
        self.declare_parameter("rate_hz", 5.0)
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True )
        
        self.xs = []
        self.ys = []
        
        #callback
        self.gps_topic = self.get_parameter("gps_topic").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        
        #state
        self._lock = threading.Lock()
        self.uav_gps = None
        
        # Publisher & Subscribers
        self.create_subscription(NavSatFix, self.gps_topic, self._gps_cb, qos_profile_sensor_data)
        
        #Time Loop
        self.timer = self.create_timer(1.0 / self.rate_hz, self.loop)
        
        self.get_logger().info(f"[GPSCheck_node] started")
        
        plt.ion()
        self.fig, self.ax = plt.subplots()
        
    def _gps_cb(self, msg: NavSatFix):
        with self._lock:
            self.uav_gps = (msg.latitude, msg.longitude)
        
    def loop(self):
        try:
            with self._lock:
                if self.uav_gps is None:
                    return
                lat,lon = self.uav_gps
            
            x, y = self.transformer.transform(lon, lat)
            
            self.xs.append(x)
            self.ys.append(y)
            
            self.ax.clear()
            self.ax.plot(self.xs, self.ys, 'b.-')
            self.ax.set_xlabel('East[m]')
            self.ax.set_ylabel('North[m]')
            self.ax.set_title('GPS Accuracy')
            plt.pause(0.001)
        except Exception as e:
            self.get_logger().error(f"[GPSCheck_node] loop error: {e}")
            
    def summary(self):
        
        x0, y0 = np.mean(self.xs), np.mean(self.ys)
        errors = np.sqrt((np.array(self.xs)-x0)**2 + (np.array(self.ys)-y0)**2)
        self.get_logger().info(f"-----GPS ACCURACY REPORT-----")
        self.get_logger().info(f"sample :{len(errors)}")
        self.get_logger().info(f"RMS(m) :{np.sqrt(np.mean(errors**2)):.2f}")
        self.get_logger().info(f"STD(m) :{np.std(errors):.2f}")
        self.get_logger().info(f"CEP50(m) :{np.percentile(errors, 50):.2f}")
        self.get_logger().info(f"CEP95(m) :{np.percentile(errors, 95):.2f}")
        
                
def main(args=None):
    rclpy.init(args=args)
    node = GPSCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
                
                
                
                
                
        
        
    
