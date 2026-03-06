#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import math
import threading
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data

try:
    from geopy.distance import geodesic
    _HAS_GEOPY = True
except Exception:
    _HAS_GEOPY = False

R_EARTH = 6378137.0  # meters


def fast_offset_to_latlon(lat0, lon0, dx, dy):
    lat0_rad = math.radians(lat0)
    dlat = dy / R_EARTH
    dlon = dx / (R_EARTH * math.cos(lat0_rad))
    lat = math.degrees(dlat + math.radians(lat0))
    lon = math.degrees(dlon + math.radians(lon0))
    return lat, lon


def geopy_offset_to_latlon(lat0, lon0, dx, dy):
    if not _HAS_GEOPY:
        return fast_offset_to_latlon(lat0, lon0, dx, dy)
    origin = (lat0, lon0)
    p_north = geodesic(meters=dy).destination(origin, 0)
    p_final = geodesic(meters=dx).destination((p_north.latitude, p_north.longitude), 90)
    return p_final.latitude, p_final.longitude


def pixel_to_camera_vector(cx, cy, fx, fy, cx0, cy0):
    x = (float(cx) - float(cx0)) / float(fx)
    y = (float(cy) - float(cy0)) / float(fy)
    z = 1.0
    return [x, y, z]


def rotation_body_to_world(vec, roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R00 = cy*cp; R01 = cy*sp*sr - sy*cr; R02 = cy*sp*cr + sy*sr
    R10 = sy*cp; R11 = sy*sp*sr + cy*cr; R12 = sy*sp*cr - cy*sr
    R20 = -sp;   R21 = cp*sr;            R22 = cp*cr
    xw = R00*vec[0] + R01*vec[1] + R02*vec[2]
    yw = R10*vec[0] + R11*vec[1] + R12*vec[2]
    zw = R20*vec[0] + R21*vec[1] + R22*vec[2]
    return [xw, yw, zw]


def vector_to_ground_offset(vec_w, altitude_m):
    vz = vec_w[2]
    if abs(vz) < 1e-9:
        return None, None
    scale = -altitude_m / vz
    dx, dy = vec_w[0] * scale, vec_w[1] * scale
    return dx, dy


class GeolocateNode(Node):
    def __init__(self):
        super().__init__("geolocate_node")

        # Parameters
        self.declare_parameter("cam_width", 640)
        self.declare_parameter("cam_height", 480)
        self.declare_parameter("fx", 641.85926898)
        self.declare_parameter("fy", 641.22254895)
        self.declare_parameter("cx", self.get_parameter("cam_width").value / 2.0)
        self.declare_parameter("cy", self.get_parameter("cam_height").value / 2.0)
        self.declare_parameter("conf_threshold", 0.0)
        self.declare_parameter("ignore_attitude_when_nadir", True)
        self.declare_parameter("use_geopy_for_offset", False)
        self.declare_parameter("output_topic", "/detections_geo")
        self.declare_parameter("input_dets_topic", "/yolo/detections")
        self.declare_parameter("gps_topic", "/mavros/global_position/global")
        self.declare_parameter("relalt_topic", "/mavros/global_position/rel_alt")
        self.declare_parameter("local_pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("round_decimals", 6)

        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.ignore_attitude_when_nadir = bool(self.get_parameter("ignore_attitude_when_nadir").value)
        self.use_geopy = bool(self.get_parameter("use_geopy_for_offset").value)
        self.input_topic = self.get_parameter("input_dets_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.gps_topic = self.get_parameter("gps_topic").value
        self.relalt_topic = self.get_parameter("relalt_topic").value
        self.local_pose_topic = self.get_parameter("local_pose_topic").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.round_decimals = int(self.get_parameter("round_decimals").value)

        # State
        self._lock = threading.Lock()
        self.uav_gps = None
        self.uav_alt = None
        self.uav_roll = 0.0
        self.uav_pitch = 0.0
        self.uav_yaw = 0.0
        self.latest_detections = []
        self._new_flag = False

        # Publisher & Subscribers
        self._pub = self.create_publisher(String, self.output_topic, 10)
        self.create_subscription(NavSatFix, self.gps_topic, self._gps_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, self.relalt_topic, self._relalt_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.local_pose_topic, self._pose_cb, qos_profile_sensor_data)
        self.create_subscription(String, self.input_topic, self._detections_cb, 10)

        # Timer loop
        self.timer = self.create_timer(1.0 / self.rate_hz, self.loop)

        self.get_logger().info(f"[geolocate_node] started: input={self.input_topic} output={self.output_topic}")

    def _gps_cb(self, msg: NavSatFix):
        with self._lock:
            self.uav_gps = (msg.latitude, msg.longitude)

    def _relalt_cb(self, msg: Float64):
        with self._lock:
            self.uav_alt = float(msg.data)

    def _pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        with self._lock:
            self.uav_roll, self.uav_pitch, self.uav_yaw = r, p, y

    def _detections_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"[geolocate_node] failed to parse detection JSON: {e}")
            return
        dets = payload.get("detections", payload if isinstance(payload, list) else [payload])
        if self.conf_threshold > 0.0:
            dets = [d for d in dets if float(d.get("confidence", d.get("conf", 1.0))) >= self.conf_threshold]
        with self._lock:
            self.latest_detections = dets
            self._new_flag = True

    def loop(self):
        try:
            with self._lock:
                new_flag = self._new_flag
                dets = list(self.latest_detections)
                self._new_flag = False
                gps, alt = self.uav_gps, self.uav_alt
                roll, pitch, yaw = self.uav_roll, self.uav_pitch, self.uav_yaw

            if not new_flag or not dets or gps is None or alt is None:
                return

            out_list = []

            for det in dets:
                pc = det.get("pixel_center")
                if pc and len(pc) >= 2:
                    cx, cy = pc[0], pc[1]
                else:
                    bbox = det.get("bbox")
                    if bbox and len(bbox) >= 4:
                        cx = (bbox[0] + bbox[2]) / 2.0
                        cy = (bbox[1] + bbox[3]) / 2.0
                    else:
                        cx, cy = det.get("cx"), det.get("cy")
                        if cx is None or cy is None:
                            continue

                # Use the sensor snapshot embedded by detection_node at frame-capture time.
                # This ensures GPS/altitude/attitude all refer to the same instant,
                # eliminating the ~1-2 m timing error from processing delay.
                det_gps_raw = det.get("uav_gps")
                use_gps   = (det_gps_raw[0], det_gps_raw[1]) if (det_gps_raw and len(det_gps_raw) == 2) else gps
                use_alt   = det.get("uav_alt") if det.get("uav_alt") is not None else alt
                use_roll  = det.get("uav_roll",  roll)
                use_pitch = det.get("uav_pitch", pitch)
                use_yaw   = det.get("uav_yaw",   yaw)

                if use_alt is None or use_alt <= 0.5:
                    continue  # skip if altitude not yet valid

                cam_vec = pixel_to_camera_vector(cx, cy, self.fx, self.fy, self.cx, self.cy)
                # Negate Z: camera optical axis points into scene (downward for nadir).
                # ENU world Z points up, so we must flip Z before applying any rotation.
                cam_vec_w = [cam_vec[0], cam_vec[1], -cam_vec[2]]
                if self.ignore_attitude_when_nadir:
                    # Apply only yaw (assume flat hover); ignore roll/pitch.
                    # Without yaw, image-right would always be mapped to world-East
                    # regardless of drone heading, causing ~10 m errors at 30 m altitude.
                    world_vec = rotation_body_to_world(cam_vec_w, 0.0, 0.0, use_yaw)
                else:
                    world_vec = rotation_body_to_world(cam_vec_w, use_roll, use_pitch, use_yaw)

                dx_m, dy_m = vector_to_ground_offset(world_vec, use_alt)
                if dx_m is None or dy_m is None:
                    continue

                lat, lon = (geopy_offset_to_latlon(use_gps[0], use_gps[1], dx_m, dy_m)
                            if self.use_geopy and _HAS_GEOPY
                            else fast_offset_to_latlon(use_gps[0], use_gps[1], dx_m, dy_m))

                det_out = dict(det)
                det_out["geo"] = [round(lat, self.round_decimals),
                                  round(lon, self.round_decimals)]
                out_list.append(det_out)

            if out_list:
                msg = String()
                msg.data = json.dumps({"detections": out_list}, ensure_ascii=False)
                self._pub.publish(msg)
                self.get_logger().info(f"[geolocate_node] published {len(out_list)} detections")

        except Exception as e:
            self.get_logger().error(f"[geolocate_node] loop error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GeolocateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

