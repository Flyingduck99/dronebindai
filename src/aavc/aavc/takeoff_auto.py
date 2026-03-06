#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 (Jazzy) rclpy node for ArduPilot via MAVROS2:
- Ensure MAVROS2 is running and connected to FCU
- Set mode GUIDED
- Arm
- Takeoff to target AGL altitude

Usage (after installing in your ROS2 pkg):
  ros2 run your_pkg auto_takeoff_ros2 --ros-args \
    -p takeoff_alt:=10.0 -p arm_timeout:=20.0 -p mode_timeout:=15.0 -p takeoff_timeout:=60.0 \
    -r /mavros:=/mavros

Note:
- This script **does not** start MAVROS2; it only uses existing /mavros.* services/topics.
- Test first in SITL. Remove props for bench tests.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

class AutoTakeoffROS2(Node):
    def __init__(self):
        super().__init__('auto_guided_arm_takeoff_ros2')
        # Parameters
        self.declare_parameter('takeoff_alt', 10.0)
        self.declare_parameter('arm_timeout', 20.0)
        self.declare_parameter('mode_timeout', 15.0)
        self.declare_parameter('takeoff_timeout', 60.0)
        self.declare_parameter('retry_rate_hz', 2.0)

        self.takeoff_alt = float(self.get_parameter('takeoff_alt').value)
        self.arm_timeout = float(self.get_parameter('arm_timeout').value)
        self.mode_timeout = float(self.get_parameter('mode_timeout').value)
        self.takeoff_timeout = float(self.get_parameter('takeoff_timeout').value)
        self.retry_rate_hz = float(self.get_parameter('retry_rate_hz').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)

        self.current_state = State()
        self.rel_alt = None

        # Subscriptions
        self.sub_state = self.create_subscription(State, '/mavros/state', self._state_cb, qos)
        self.sub_rel_alt = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self._rel_alt_cb, qos_rel)

        # Clients
        self.cli_set_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.get_logger().info('Waiting for MAVROS2 services...')
        for cli, name in [
            (self.cli_set_mode, '/mavros/set_mode'),
            (self.cli_arm, '/mavros/cmd/arming'),
            (self.cli_takeoff, '/mavros/cmd/takeoff'),
        ]:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f'Service not available: {name}')
                raise SystemExit(1)
        self.get_logger().info('Services are available.')

        # Kick off sequence
        self.create_timer(0.1, self._bootstrap_once)
        self._started = False

    def _bootstrap_once(self):
        if self._started:
            return
        self._started = True
        self.get_logger().info('Starting GUIDED → ARM → TAKEOFF sequence...')
        ok = self.wait_for_connection(30.0)
        ok = ok and self.ensure_mode('GUIDED')
        ok = ok and self.ensure_armed()
        ok = ok and self.do_takeoff()
        if ok:
            self.get_logger().info('Takeoff complete ✅')
        else:
            self.get_logger().error('Sequence failed ❌')

    # ---------- Callbacks ----------
    def _state_cb(self, msg: State):
        self.current_state = msg

    def _rel_alt_cb(self, msg: Float64):
        self.rel_alt = msg.data

    # ---------- Helpers ----------
    def sleep(self, sec):
        rclpy.spin_once(self, timeout_sec=max(0.001, 1.0/self.retry_rate_hz))
        time.sleep(sec)

    def wait_for_connection(self, timeout):
        start = time.time()
        while rclpy.ok():
            if self.current_state.connected:
                self.get_logger().info('FCU connected via MAVROS2.')
                return True
            if time.time() - start > timeout:
                self.get_logger().error('Timeout waiting for FCU connection.')
                return False
            self.sleep(1.0/self.retry_rate_hz)

    def ensure_mode(self, mode_name: str):
        start = time.time()
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_name
        while rclpy.ok():
            if self.current_state.mode == mode_name:
                self.get_logger().info(f'Mode already {mode_name}')
                return True
            self.cli_set_mode.call_async(req)
            self.get_logger().info(f'Trying to set mode → {mode_name}')
            if time.time() - start > self.mode_timeout:
                self.get_logger().error(f'Timeout setting mode to {mode_name}')
                return False
            self.sleep(1.0/self.retry_rate_hz)

    def ensure_armed(self):
        start = time.time()
        req = CommandBool.Request()
        req.value = True
        while rclpy.ok():
            if self.current_state.armed:
                self.get_logger().info('Vehicle already armed')
                return True
            self.cli_arm.call_async(req)
            self.get_logger().info('Trying to arm')
            if time.time() - start > self.arm_timeout:
                self.get_logger().error('Timeout arming vehicle')
                return False
            self.sleep(1.0/self.retry_rate_hz)

    def do_takeoff(self):
        req = CommandTOL.Request()
        # For ArduPilot in GUIDED: altitude is AGL; lat/lon can be NaN to use current
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = float('nan')
        req.longitude = float('nan')
        req.altitude = self.takeoff_alt
        future = self.cli_takeoff.call_async(req)
        self.get_logger().info(f'Sent takeoff command to {self.takeoff_alt:.1f} m AGL')

        start = time.time()
        # Wait until ~95% of target alt
        while rclpy.ok():
            if self.rel_alt is not None:
                if self.rel_alt >= 0.95 * self.takeoff_alt:
                    self.get_logger().info(f'Reached target altitude ({self.rel_alt:.2f}/{self.takeoff_alt:.2f} m).')
                    return True
                else:
                    self.get_logger().info(f'RelAlt: {self.rel_alt:.2f} m')
            if time.time() - start > self.takeoff_timeout:
                self.get_logger().error('Timeout waiting for takeoff altitude')
                return False
            self.sleep(0.5)


def main():
    rclpy.init()
    node = AutoTakeoffROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

