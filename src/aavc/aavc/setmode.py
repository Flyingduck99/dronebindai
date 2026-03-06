#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint

class MavrosCommander(Node):
    def __init__(self):
        super().__init__('mavros_commander')
        self.main_menu()

    def run_cmd(self, cmd):
        print(f"\n>>> Running: {cmd}")
        os.system(cmd)

    def change_mode(self):
        while True:
            print("\n===== CHANGE MODE =====")
            print("1. Guided")
            print("2. Auto")
            print("3. RTL")
            print("4. Land")
            print("0. Back")
            mode_choice = input("Select mode: ")

            mode_map = {
                "1": "GUIDED",
                "2": "AUTO",
                "3": "RTL",
                "4": "LAND"
            }

            if mode_choice == "0":
                break
            elif mode_choice in mode_map:
                mode = mode_map[mode_choice]
                self.run_cmd(f"ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{{base_mode: 0, custom_mode: \"{mode}\"}}'")
            else:
                print("Invalid choice.")

    def gripper_menu(self):
        while True:
            print("\n===== GRIPPER =====")
            print("1. Grab")
            print("2. Release")
            print("0. Back")
            g_choice = input("Select: ")

            if g_choice == "0":
                break
            elif g_choice == "1":
                self.run_cmd("ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong '{command: 211, param1: 1, param2: 1}'")
            elif g_choice == "2":
                self.run_cmd("ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong '{command: 211, param1: 0, param2: 0}'")
            else:
                print("Invalid choice.")

    def arm_disarm(self):
        choice = input("Enter '1' to ARM or '0' to DISARM: ")
        if choice == '1':
            self.run_cmd("ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'")
        elif choice == '0':
            self.run_cmd("ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: false}'")
        else:
            print("Invalid choice.")

    def reboot(self):
        self.run_cmd("ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong '{command: 246, param1: 1}'")

    def upload_mission(self):
        folder = "/home/ciimav/ros2_ws/src/aavc/aavc/waypoint"  # โฟลเดอร์ที่เก็บไฟล์ mission
        files = [f for f in os.listdir(folder) if f.endswith('.waypoints')]
        if not files:
            print("No waypoint files found!")
            return

        print("\nAvailable mission files:")
        for i, f in enumerate(files):
            print(f"{i+1}. {f}")

        try:
            idx = int(input("Select file: ")) - 1
            if not (0 <= idx < len(files)):
                print("Invalid selection.")
                return

            filepath = os.path.join(folder, files[idx])
            print(f"\nLoading mission from {filepath}")

            # อ่านไฟล์ .waypoints และสร้าง list ของ Waypoint
            waypoints = []
            with open(filepath, 'r') as f:
                lines = f.readlines()
                for line in lines:
                  if line.startswith("#") or line.startswith("QGC") or not line.strip():
                      continue
                  parts = line.strip().split('\t')
                  
                  # ข้ามช่อง index แรก เช่น 0
                  parts = parts[1:]
                
                  wp = Waypoint()
                  wp.is_current = bool(int(parts[0]))
                  wp.frame = int(parts[1])
                  wp.command = int(parts[2])
                  wp.param1 = float(parts[3])
                  wp.param2 = float(parts[4])
                  wp.param3 = float(parts[5])
                  wp.param4 = float(parts[6])
                  wp.x_lat = float(parts[7])
                  wp.y_long = float(parts[8])
                  wp.z_alt = float(parts[9])
                  wp.autocontinue = bool(int(parts[10]))
                
                  waypoints.append(wp)


            # เรียก service /mavros/mission/push
            client = self.create_client(WaypointPush, '/mavros/mission/push')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for /mavros/mission/push service...')

            req = WaypointPush.Request()
            req.start_index = 0
            req.waypoints = waypoints

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                print("Mission uploaded successfully!")
            else:
                print("Failed to upload mission.")

        except ValueError:
            print("Invalid input.")

    def main_menu(self):
        while True:
            print("\n========== MAVROS COMMANDER ==========")
            print("1. Change Mode")
            print("2. Takeoff")
            print("3. Set Stream Rate (10Hz)")
            print("4. Upload Mission")
            print("5. Reboot Pixhawk")
            print("6. Gripper")
            print("7. Arm / Disarm")
            print("8. Go to GPS coordinate")
            print("0. Exit")

            choice = input("Select: ")

            if choice == "1":
                self.change_mode()
            elif choice == "2":
                alt = input("Enter takeoff altitude (m): ")
                self.run_cmd(f"ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL '{{min_pitch: 0.0, yaw: 0.0, latitude: nan, longitude: nan, altitude: {alt}}}'")
            elif choice == "3":
                self.run_cmd("ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate '{stream_id: 0, message_rate: 10, on_off: true}'")
            elif choice == "4":
                self.upload_mission()  # <-- เรียกฟังก์ชัน upload mission ใหม่
            elif choice == "5":
                self.reboot()
            elif choice == "6":
                self.gripper_menu()
            elif choice == "7":
                self.arm_disarm()
            elif choice == "8":
                self.goto_gps()
            elif choice == "0":
                print("Prepare to Shutting Down...")
                print("Shutdown complete bye")
                break
            else:
                print("Invalid choice, try again.")
                
    def goto_gps(self):
      print("\n===== GO TO GPS COORDINATE =====")
      try:
          lat = float(input("Enter latitude: "))
          lon = float(input("Enter longitude: "))
          alt = float(input("Enter altitude (m): "))
      except ValueError:
          print("Invalid input. Please enter numeric values.")
          return
    
      # ใช้ GeoPoseStamped เพื่อส่งพิกัด GPS
      from geographic_msgs.msg import GeoPoseStamped
      import time
    
      pub = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 10)
    
      msg = GeoPoseStamped()
      msg.pose.position.latitude = lat
      msg.pose.position.longitude = lon
      msg.pose.position.altitude = alt
    
      print(f"\nSending target: lat={lat}, lon={lon}, alt={alt}")
      print("Make sure drone is in GUIDED mode and armed!")
    
      for i in range(10):  # ส่งซ้ำ 10 ครั้ง เพื่อความมั่นใจ
          pub.publish(msg)
          time.sleep(0.5)
    
      print("? Target GPS position published.")
            

def main():
    rclpy.init()
    node = MavrosCommander()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
