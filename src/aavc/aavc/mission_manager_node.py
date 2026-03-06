#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import String
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
import math
import threading


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')
        # ---------------- QoS ----------------
        qos_sub_default = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_pub_volatile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Publishers ---
        self.setpoint_pub      = self.create_publisher(GeoPoseStamped, "/mavros/setpoint_position/global", 10)
        self.arrival_pub       = self.create_publisher(String, "/mission/arrived_at_clusters", 10)
        # Publishes a human-readable warning when clusters are ready, asking operator to confirm.
        self.guidance_req_pub  = self.create_publisher(String, "/mission/guidance_request", 10)

        # --- Subscribers ---
        self.create_subscription(String, "/detections/clusterssum", self.cluster_callback, 10)
        self.create_subscription(String, "/validation/results", self.validation_callback, 10)
        # Operator sends "go" (or "yes") here to confirm GUIDED navigation to clusters.
        self.create_subscription(String, "/mission/confirm_guided", self.confirm_guided_callback, 10)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self.velocity_callback, qos_sub_default)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.altitude_callback, qos_sub_default)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.gps_callback, qos_sub_default)

        # --- Service Clients ---
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arming_client   = self.create_client(CommandBool, "/mavros/cmd/arming")

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /mavros/set_mode service...")
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /mavros/cmd/arming service...")

        # --- Mission State ---
        self.current_mode = None
        self.clusters = []
        self.current_cluster_idx = 0
        self.waiting_validation = False
        self.descend_to_10m = False
        self.current_wp = None
        self.publish_wp = False
        self.arrived_published = False
        self.started_moving = False
        self.final_yes = False

        # --- Velocity / Altitude Tracking ---
        self.current_vel = None
        self.speed_xy_history = []
        self.speed_history_len = 10
        self.speed_threshold = 0.1  # m/s
        self.current_altitude = None

        # --- Descend State (non-blocking) ---
        self.descend_active = False
        self.descend_target_alt = 10.0  # AGL (matches local_position/pose.position.z)

        # --- Takeoff AMSL altitude (captured from first GPS fix) ---
        self.home_alt_amsl = None
        self.nav_alt_agl = 30.0  # desired navigation altitude AGL

        # --- Guided confirmation state ---
        # True while waiting for operator to confirm GUIDED navigation.
        self.awaiting_guided_confirm = False
        self._confirm_reminder_ticks = 0   # counts logic ticks for periodic reminder log
        # Stdin thread sets this; logic_timer_cb processes it on the ROS thread.
        self._pending_confirm_cmd = None
        self._stdin_thread = None

        self.get_logger().info("[MISSION] Mission Manager Node started")

        # --- Arm + AUTO (called in __init__, before rclpy.spin, so blocking is safe) ---
        self.arm_drone(True)
        future = self.set_mode("AUTO")
        rclpy.spin_until_future_complete(self, future)

        # --- Timers ---
        # 1) สตรีม setpoint 10 Hz (แก้ประเด็น 6) ให้ต่อเนื่อง/ไม่เริ่มช้า)
        self.setpoint_timer = self.create_timer(0.1, self.setpoint_timer_cb)

        # 2) Main logic tick 10 Hz (เช็ค arrival / จัดการ descend แบบ non-blocking)
        self.logic_timer = self.create_timer(0.1, self.logic_timer_cb)

    # --- Service functions ---
    def set_mode(self, mode_name, on_done=None):
        """
        Async-safe mode change. on_done() is called after the service confirms success.
        Using spin_until_future_complete inside a subscription callback would deadlock
        the single-threaded executor, so we use add_done_callback instead.
        """
        req = SetMode.Request()
        req.custom_mode = mode_name
        future = self.set_mode_client.call_async(req)

        def _cb(f):
            res = f.result()
            if res and res.mode_sent:
                self.get_logger().info(f"[MISSION] Flight mode changed to {mode_name}")
                self.current_mode = mode_name
            else:
                self.get_logger().warn(f"[MISSION] Failed to change mode to {mode_name}")
                # Reset pending state so cluster_callback can retry
                if self.current_mode == "GUIDED_PENDING":
                    self.current_mode = None
            if on_done:
                on_done()

        future.add_done_callback(_cb)
        return future  # caller may block on this if called before rclpy.spin

    def arm_drone(self, arm=True):
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)  # safe: called in __init__ before spin
        res = future.result()
        if res and res.success:
            self.get_logger().info(f"[MISSION] Drone armed: {arm}")

    # --- Callbacks ---
    def velocity_callback(self, msg: TwistStamped):
        self.current_vel = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        if not self.started_moving:
            vx, vy, _ = self.current_vel
            speed_xy = math.sqrt(vx**2 + vy**2)
            if speed_xy > 0.5:
                self.started_moving = True

    def altitude_callback(self, msg: PoseStamped):
        self.current_altitude = msg.pose.position.z  # AGL from takeoff point (ENU local Z)

    def gps_callback(self, msg: NavSatFix):
        # Capture takeoff AMSL altitude from the first valid GPS fix.
        # GeoPoseStamped.pose.position.altitude is AMSL, so we must add the
        # field elevation to any AGL target to get the correct AMSL command.
        if self.home_alt_amsl is None and msg.altitude > 0.0:
            self.home_alt_amsl = msg.altitude
            self.get_logger().info(f"[MISSION] Home AMSL altitude captured: {self.home_alt_amsl:.1f} m")

    def cluster_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                new_clusters = data
            elif isinstance(data, dict):
                new_clusters = data.get("clustersum", [])
            else:
                self.get_logger().warn("[CLUSTER] Unexpected format")
                return

            self.get_logger().info(f"[CLUSTER] Received {len(new_clusters)} clusters")
            self.clusters.extend(new_clusters)

            # Only prompt once; ignore if already waiting/flying.
            if (self.clusters
                    and not self.awaiting_guided_confirm
                    and self.current_mode not in ("GUIDED", "GUIDED_PENDING")
                    and not self.waiting_validation):
                self.awaiting_guided_confirm = True
                self._confirm_reminder_ticks = 0
                self._publish_guidance_request()

        except Exception as e:
            self.get_logger().error(f"[CLUSTER] Failed to parse cluster: {e}")

    def _publish_guidance_request(self):
        """Publish a warning summary and ask operator to confirm GUIDED navigation."""
        summary = []
        for i, c in enumerate(self.clusters):
            lat, lon = c.get("center", [0, 0])
            summary.append(f"  [{i}] lat={lat:.6f} lon={lon:.6f} "
                           f"label={c.get('color','?')} count={c.get('items_estimated','?')}")
        summary_str = "\n".join(summary)

        self.get_logger().warn(
            f"\n"
            f"========================================\n"
            f"  [MISSION] CLUSTERS READY — WAITING FOR OPERATOR CONFIRMATION\n"
            f"  {len(self.clusters)} cluster(s) found:\n"
            f"{summary_str}\n"
            f"  >> Type 'go' and press Enter to start GUIDED navigation.\n"
            f"  >> Type 'cancel' and press Enter to abort.\n"
            f"========================================"
        )

        req_msg = String()
        req_msg.data = json.dumps({
            "status": "awaiting_confirm",
            "cluster_count": len(self.clusters),
            "clusters": [
                {"id": i,
                 "center": c.get("center"),
                 "label": c.get("color"),
                 "count": c.get("items_estimated")}
                for i, c in enumerate(self.clusters)
            ]
        })
        self.guidance_req_pub.publish(req_msg)

        # Start stdin reader thread if not already running.
        if self._stdin_thread is None or not self._stdin_thread.is_alive():
            self._stdin_thread = threading.Thread(
                target=self._stdin_reader, daemon=True, name="stdin_confirm"
            )
            self._stdin_thread.start()

    def _stdin_reader(self):
        """
        Background daemon thread: reads keyboard input and sets _pending_confirm_cmd.
        Runs until a valid command is entered or awaiting_guided_confirm goes False.
        The actual ROS actions are executed on the main thread in logic_timer_cb.
        """
        while self.awaiting_guided_confirm:
            try:
                print("[MISSION] >> ", end="", flush=True)
                line = input()
                cmd = line.strip().lower()
                if cmd in ("go", "yes", "confirm", "cancel"):
                    self._pending_confirm_cmd = cmd
                    break
                elif cmd:
                    print(f"[MISSION] Unknown command '{cmd}'. Type 'go' to confirm or 'cancel' to abort.")
            except (EOFError, KeyboardInterrupt):
                break

    def confirm_guided_callback(self, msg: String):
        """Operator sends 'go'/'yes' to confirm, or 'cancel' to abort GUIDED navigation."""
        if not self.awaiting_guided_confirm:
            return

        cmd = msg.data.lower().strip()
        if cmd in ("go", "yes", "confirm"):
            self.get_logger().info("[MISSION] Operator confirmed -> switching to GUIDED mode")
            self.awaiting_guided_confirm = False
            self.current_mode = "GUIDED_PENDING"
            self.set_mode("GUIDED", on_done=self.goto_next_cluster)
        elif cmd == "cancel":
            self.get_logger().warn("[MISSION] Operator cancelled GUIDED navigation -> RTL")
            self.awaiting_guided_confirm = False
            self.clusters = []
            self.set_mode("RTL")
        else:
            self.get_logger().warn(
                f"[MISSION] Unknown confirm command '{cmd}'. "
                f"Send 'go' to confirm or 'cancel' to abort."
            )

    def validation_callback(self, msg: String):
        # Only accept validation after the drone has confirmed arrival.
        # Without this guard, a stale or early "yes" would trigger descent
        # while the drone is still flying to the cluster.
        if not self.waiting_validation or not self.arrived_published:
            return
        result = msg.data.lower().strip()
        last_cluster = (self.current_cluster_idx == len(self.clusters) - 1)

        if result == "yes":
            if last_cluster:
                self.get_logger().info("[VALIDATION] YES on final cluster -> Descend to 10m then RTL")
                self.descend_to_10m = True
                self.final_yes = True
            else:
                self.get_logger().info("[VALIDATION] YES -> Descend to 10m then continue")
                self.descend_to_10m = True
        else:
            if last_cluster:
                self.get_logger().info("[VALIDATION] NO on final cluster -> RTL immediately")
                self.set_mode("RTL")
                self.waiting_validation = False
            else:
                self.get_logger().info("[VALIDATION] NO -> Continue mission")
                self.waiting_validation = False
                self.current_cluster_idx += 1
                self.goto_next_cluster()

    # --- Mission helpers ---
    def goto_next_cluster(self):
        if self.current_cluster_idx < len(self.clusters):
            cluster = self.clusters[self.current_cluster_idx]
            self.get_logger().info(f"[MISSION] Going to cluster {self.current_cluster_idx} at {cluster['center']}")

            wp = GeoPoseStamped()
            wp.header.frame_id = "map"
            wp.pose.position.latitude  = cluster['center'][0]
            wp.pose.position.longitude = cluster['center'][1]
            # altitude in GeoPoseStamped is AMSL. Add home elevation so the drone
            # maintains nav_alt_agl above the ground regardless of field elevation.
            nav_alt_amsl = (self.home_alt_amsl + self.nav_alt_agl) if self.home_alt_amsl else self.nav_alt_agl
            wp.pose.position.altitude  = nav_alt_amsl

            self.current_wp = wp
            self.publish_wp = True
            self.waiting_validation = True
            self.descend_to_10m = False
            self.arrived_published = False
            self.started_moving = False
            self.final_yes = False
        else:
            self.get_logger().info("[MISSION] All clusters processed.")
            self.publish_wp = False

    def check_arrival(self):
        if not self.current_wp or self.current_vel is None or self.arrived_published:
            return False

        vx, vy, _ = self.current_vel
        speed_xy = math.sqrt(vx**2 + vy**2)

        self.speed_xy_history.append(speed_xy)
        if len(self.speed_xy_history) > self.speed_history_len:
            self.speed_xy_history.pop(0)

        avg_speed_xy = sum(self.speed_xy_history) / len(self.speed_xy_history)

        if self.started_moving and avg_speed_xy < self.speed_threshold:
            if not self.arrived_published:
                msg = String()
                msg.data = json.dumps({
                    "cluster_id": self.current_cluster_idx,
                    "status": "arrived"
                })
                self.arrival_pub.publish(msg)
                self.get_logger().info(f"[MISSION] Arrived at cluster {self.current_cluster_idx} (avg_speed_xy={avg_speed_xy:.3f} m/s)")
                self.arrived_published = True
            return True
        return False

    # --- Non-blocking descend manager (แทน while-loop เดิม) ---
    def handle_descend(self):
        # ถูกเรียกใน logic_timer_cb เมื่อ self.descend_active=True
        if not self.current_wp or self.current_altitude is None:
            return

        target_alt = self.descend_target_alt
        current_alt = self.current_altitude

        # ปรับ target alt ลงไปที่ 10 m แล้วปล่อยให้ setpoint_timer สตรีมต่อเนื่อง
        self.current_wp.pose.position.altitude = target_alt

        if current_alt <= target_alt + 0.1:
            # จบการลดระดับ
            self.get_logger().info("[MISSION] Descend complete")
            self.descend_active = False

            if self.final_yes:
                self.get_logger().info("[MISSION] Final cluster descend done -> RTL")
                self.set_mode("RTL")
                self.waiting_validation = False
            else:
                self.get_logger().info("[MISSION] Continue to next cluster")
                self.waiting_validation = False
                self.current_cluster_idx += 1
                self.goto_next_cluster()

    # --- Timers ---
    def setpoint_timer_cb(self):
        """
        ยิง setpoint ต่อเนื่องที่ 10 Hz เพื่อแก้ปัญหา 6) สตรีมเบาบาง/เริ่มช้า
        """
        if self.publish_wp and self.current_wp:
            # ประทับเวลาและส่งออกทุกครั้งที่ timer ติด
            self.current_wp.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.current_wp)

    def logic_timer_cb(self):
        """
        ติ๊กหลักสำหรับเช็ค arrival และจัดการ descend แบบไม่บล็อก
        """
        # Process stdin command submitted by _stdin_reader thread.
        if self._pending_confirm_cmd is not None:
            cmd = self._pending_confirm_cmd
            self._pending_confirm_cmd = None
            fake_msg = String()
            fake_msg.data = cmd
            self.confirm_guided_callback(fake_msg)

        # Remind operator every 10 s (100 ticks @ 10 Hz) while waiting for confirmation.
        if self.awaiting_guided_confirm:
            self._confirm_reminder_ticks += 1
            if self._confirm_reminder_ticks >= 100:
                self._confirm_reminder_ticks = 0
                self._publish_guidance_request()
            return  # hold all other logic until operator confirms

        # arrival check ระหว่างรอ validation
        if self.waiting_validation:
            self.check_arrival()

        # trigger descend (เปิดโหมดทำงานแบบไม่บล็อก)
        if self.descend_to_10m and not self.descend_active:
            self.get_logger().info(f"[MISSION] Descending to {self.descend_target_alt} m for cluster {self.current_cluster_idx}")
            self.descend_active = True
            self.descend_to_10m = False

        # ถ้ากำลัง descend อยู่ให้จัดการต่อเนื่อง
        if self.descend_active:
            self.handle_descend()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)  # ใช้ spin + timers แทนลูป while เดิม
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

