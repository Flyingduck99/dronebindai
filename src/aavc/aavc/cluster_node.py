#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import json
import numpy as np
from collections import Counter
from std_msgs.msg import String
from mavros_msgs.msg import WaypointList, WaypointReached
from sklearn.cluster import DBSCAN
from geopy.distance import geodesic


class ClusterNode(Node):
    def __init__(self):
        super().__init__('cluster_node')

        # --- Params ---
        self.declare_parameter("eps_meters", 15.0)
        self.declare_parameter("min_samples", 2)
        self.declare_parameter("consensus_ratio", 0.60)
        self.declare_parameter("min_frames", 2)
        self.declare_parameter("round_decimals", 6)
        self.declare_parameter("use_frame_seq_priority", True)

        self.declare_parameter("priority_color", "undefined")
        self.declare_parameter("priority_count", 2)

        self.eps_meters = float(self.get_parameter("eps_meters").value)
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.consensus_ratio = float(self.get_parameter("consensus_ratio").value)
        self.min_frames = int(self.get_parameter("min_frames").value)
        self.round_decimals = int(self.get_parameter("round_decimals").value)
        self.use_frame_seq_priority = bool(self.get_parameter("use_frame_seq_priority").value)

        self.priority_color = str(self.get_parameter("priority_color").value)
        self.priority_count = int(self.get_parameter("priority_count").value)

        # --- Subs ---
        self.create_subscription(String, '/detections_geo', self.detection_callback, 10)
        self.create_subscription(WaypointReached, '/mavros/mission/reached', self.wp_callback, 10)
        self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.mission_callback, 10)

        # --- Pub ---
        self.cluster_pub = self.create_publisher(String, '/detections/clusterssum', 10)

        # --- State ---
        self.all_detections = []
        self.final_wp = None
        self.final_wp_latlon = None
        self.has_published = False
        self.origin = None

        self.get_logger().info(
            f"Cluster node started (eps={self.eps_meters}, min_samples={self.min_samples}, min_frames={self.min_frames})"
        )

    # ---------- Callbacks ----------
    def detection_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"[ClusterNode] Failed to parse /detections_geo: {e}")
            return

        detections = data.get("detections", [])
        hdr = data.get("header", {}) if isinstance(data, dict) else {}

        for det in detections:
            label = det.get('label', 'unknown')
            color = det.get('color') if isinstance(det.get('color'), str) else 'undefined'
            geo = det.get('geo', None)
            if not (isinstance(geo, (list, tuple)) and len(geo) == 2):
                continue

            fk = det.get('frame_key')
            if not isinstance(fk, dict):
                # Prefer frame_stamp embedded by detection_node (actual camera capture time).
                # Without this, all detections map to sec=0/nsec=0 → frames_observed always 1
                # → consensus_ok always False and items_estimated is wrong.
                stamp = det.get('frame_stamp', {})
                fk = {
                    "sec":  int(stamp.get("sec",     hdr.get("stamp_sec",  0))),
                    "nsec": int(stamp.get("nanosec", hdr.get("stamp_nsec", 0))),
                    "seq":  int(hdr.get("frame_seq", 0)),
                    "fid":  hdr.get("frame_id", ""),
                }

            self.all_detections.append({
                'label': label,
                'color': color or 'undefined',
                'geo': [float(geo[0]), float(geo[1])],
                'frame_key': fk
            })

    def mission_callback(self, msg: WaypointList):
        if not msg.waypoints:
            return

        # --- เลือก final NAV waypoint จริง (cmd=16) ---
        final_idx = None
        for i, wp in reversed(list(enumerate(msg.waypoints))):
            if wp.command == 16 and wp.x_lat != 0.0 and wp.y_long != 0.0:
                final_idx = i
                break

        if final_idx is None:
            self.get_logger().warn("No valid NAV waypoint found for final_wp")
            return

        self.final_wp = final_idx
        wp = msg.waypoints[self.final_wp]
        self.final_wp_latlon = (wp.x_lat, wp.y_long)
        self.get_logger().info(f"Final NAV waypoint set to index {self.final_wp}, latlon={self.final_wp_latlon}")

    def wp_callback(self, msg: WaypointReached):
        if self.final_wp is None:
            return  # ไม่รู้ final waypoint → ไม่ publish

        if msg.wp_seq < self.final_wp:
            self.get_logger().info(f"Reached waypoint seq {msg.wp_seq} (not final)")
            return  # ยังไม่ถึง final NAV waypoint

        if self.has_published:
            return

        self.get_logger().info(f"Reached final NAV waypoint seq {msg.wp_seq} → Publishing clusters")
        self.publish_clusters()
        self.has_published = True

    # ---------- Helpers ----------
    def latlon_to_xy(self, geo):
        if self.origin is None:
            self.origin = geo
        lat0, lon0 = self.origin
        lat, lon = geo
        dx = geodesic((lat0, lon0), (lat0, lon)).meters
        dy = geodesic((lat0, lon0), (lat, lon0)).meters
        if lon < lon0: dx *= -1
        if lat < lat0: dy *= -1
        return [dx, dy]

    def xy_to_latlon(self, dx, dy):
        lat0, lon0 = self.origin
        p_north = geodesic(meters=dy).destination((lat0, lon0), 0)
        p_final = geodesic(meters=dx).destination((p_north.latitude, p_north.longitude), 90)
        return p_final.latitude, p_final.longitude

    def _frame_key_tuple(self, fk: dict):
        if self.use_frame_seq_priority:
            seq = int(fk.get('seq', 0)) if isinstance(fk, dict) else 0
            if seq > 0:
                return ('seq', seq)
        sec = int(fk.get('sec', 0)) if isinstance(fk, dict) else 0
        nsec = int(fk.get('nsec', 0)) if isinstance(fk, dict) else 0
        return ('time', sec, nsec)

    def _priority_tier(self, cluster):
        color = cluster.get('color')
        count = cluster.get('items_estimated')
        if color == self.priority_color and count == self.priority_count:
            return 0
        elif color == self.priority_color:
            return 1
        elif count == self.priority_count:
            return 2
        else:
            return 3

    def _distance_to_final_wp(self, cluster):
        if not self.final_wp_latlon:
            return float('inf')
        center_lat, center_lon = cluster['center']
        return geodesic((center_lat, center_lon), self.final_wp_latlon).meters

    # ---------- Core ----------
    def publish_clusters(self):
        if not self.all_detections:
            self.get_logger().info("No detections to cluster.")
            return

        cluster_results = []

        colors = set([det['color'] for det in self.all_detections])
        for color in colors:
            color_dets = [det for det in self.all_detections if det['color'] == color]

            coords_m = [self.latlon_to_xy(det['geo']) for det in color_dets]
            if len(coords_m) < self.min_samples:
                continue

            clustering = DBSCAN(
                eps=self.eps_meters,
                min_samples=self.min_samples,
                metric='euclidean'
            ).fit(np.array(coords_m))
            labels = clustering.labels_

            unique_labels = set(labels)
            unique_labels.discard(-1)

            for lbl in unique_labels:
                member_idxs = [i for i, lab in enumerate(labels) if lab == lbl]
                members = [color_dets[i] for i in member_idxs]
                member_xy = [coords_m[i] for i in member_idxs]

                center_x = float(np.mean([p[0] for p in member_xy]))
                center_y = float(np.mean([p[1] for p in member_xy]))
                center_lat, center_lon = self.xy_to_latlon(center_x, center_y)

                per_frame_points = {}
                for m in members:
                    key = self._frame_key_tuple(m.get('frame_key', {}))
                    rounded = (round(m['geo'][0], self.round_decimals),
                               round(m['geo'][1], self.round_decimals))
                    per_frame_points.setdefault(key, set()).add(rounded)

                counts = [len(s) for s in per_frame_points.values()]
                if not counts:
                    continue
                hist = Counter(counts)
                frames_observed = int(sum(hist.values()))
                max_count = max(hist.keys())
                items_estimated = int(max_count)

                support_ratio = float(hist[max_count]) / float(frames_observed) if frames_observed > 0 else 0.0
                counts_hist_dict = {str(k): int(v) for k, v in sorted(hist.items())}
                consensus_ok = (frames_observed >= self.min_frames)

                cluster_results.append({
                    'color': color,
                    'count': items_estimated,
                    'center': [center_lat, center_lon],
                    'items_estimated': items_estimated,
                    'support_ratio': round(support_ratio, 3),
                    'frames_observed': frames_observed,
                    'consensus_ok': bool(consensus_ok),
                    'counts_hist': counts_hist_dict,
                    'consensus_strategy': 'max_per_frame'
                })

        if cluster_results:
            cluster_results = sorted(
                cluster_results,
                key=lambda c: (self._priority_tier(c), self._distance_to_final_wp(c))
            )
            msg_out = String()
            msg_out.data = json.dumps(cluster_results)
            self.cluster_pub.publish(msg_out)
            self.get_logger().info(f"Published clusters (sorted): {cluster_results}")
        else:
            self.get_logger().info("No clusters found.")

        self.all_detections = []


def main(args=None):
    rclpy.init(args=args)
    node = ClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
