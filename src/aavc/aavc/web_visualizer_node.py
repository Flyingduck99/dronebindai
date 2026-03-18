#!/usr/bin/env python3
"""
web_visualizer_node.py
======================
Serves a local web dashboard (Flask, port 5000) showing:
  - Live cluster map (ENU canvas)
  - Latest annotated camera image
  - Evidence snapshots auto-saved when drone arrives at a cluster

Subscriptions:
  /detections/clusterssum       (std_msgs/String)  – JSON cluster list
  /camera/image_anno            (sensor_msgs/Image) – YOLO-annotated frame
  /mission/arrived_at_clusters  (std_msgs/String)  – arrival events

Access: http://<host-ip>:5000
"""

import json
import os
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from flask import Flask, jsonify, send_file, send_from_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ── Locate web_static via the installed share directory ────────────────────────
_STATIC_DIR = os.path.join(get_package_share_directory('strike_mission'), 'web_static')


class WebVisualizerNode(Node):

    def __init__(self):
        super().__init__('web_visualizer_node')

        # Parameters
        self.declare_parameter('web_port',   5000)
        self.declare_parameter('snapshot_dir', '/tmp/experiment/snapshots')
        self.declare_parameter('home_lat',   13.84680177)
        self.declare_parameter('home_lon',   100.5665731)

        self.port         = self.get_parameter('web_port').value
        self.snapshot_dir = self.get_parameter('snapshot_dir').value
        self.home_lat     = self.get_parameter('home_lat').value
        self.home_lon     = self.get_parameter('home_lon').value

        os.makedirs(self.snapshot_dir, exist_ok=True)

        self.bridge = CvBridge()
        self._lock  = threading.Lock()

        # Shared state (written by ROS callbacks, read by Flask)
        self._clusters     = []          # latest parsed cluster list
        self._latest_frame = None        # latest JPEG bytes
        self._snapshots    = []          # list of {filename, timestamp, cluster_id}

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            String, '/detections/clusterssum',
            self._clusters_cb, 10)

        self.create_subscription(
            Image, '/camera/image_anno',
            self._image_cb, qos_profile_sensor_data)

        self.create_subscription(
            String, '/mission/arrived_at_clusters',
            self._arrived_cb, 10)

        # ── Flask app ─────────────────────────────────────────────────────────
        self._app = Flask(__name__, static_folder=_STATIC_DIR, static_url_path='/static')
        self._register_routes()

        flask_thread = threading.Thread(
            target=lambda: self._app.run(host='0.0.0.0', port=self.port,
                                         debug=False, use_reloader=False),
            daemon=True)
        flask_thread.start()

        self.get_logger().info(
            f'Web visualizer running at http://0.0.0.0:{self.port}')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _clusters_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._clusters = data if isinstance(data, list) else []
        except Exception as e:
            self.get_logger().warn(f'cluster parse error: {e}')

    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self._lock:
                self._latest_frame = buf.tobytes()
        except Exception as e:
            self.get_logger().warn(f'image decode error: {e}')

    def _arrived_cb(self, msg: String):
        """Auto-save a snapshot when the drone arrives at a cluster."""
        try:
            data   = json.loads(msg.data)
            cid    = data.get('cluster_id', '?')
            ts     = datetime.now().strftime('%Y%m%d_%H%M%S')
            fname  = f'cluster_{cid}_{ts}.jpg'
            fpath  = os.path.join(self.snapshot_dir, fname)
            with self._lock:
                frame_bytes = self._latest_frame
            if frame_bytes:
                with open(fpath, 'wb') as f:
                    f.write(frame_bytes)
                snap = {'filename': fname, 'timestamp': ts, 'cluster_id': cid}
                with self._lock:
                    self._snapshots.append(snap)
                self.get_logger().info(f'Snapshot saved: {fpath}')
            else:
                self.get_logger().warn('No image available for snapshot.')
        except Exception as e:
            self.get_logger().warn(f'snapshot error: {e}')

    # ── Flask routes ──────────────────────────────────────────────────────────

    def _register_routes(self):
        app = self._app

        @app.route('/')
        def index():
            with open(os.path.join(_STATIC_DIR, 'index.html')) as f:
                return f.read()

        @app.route('/api/clusters')
        def api_clusters():
            with self._lock:
                return jsonify({
                    'clusters':  self._clusters,
                    'home_lat':  self.home_lat,
                    'home_lon':  self.home_lon,
                    'timestamp': time.time(),
                })

        @app.route('/api/image')
        def api_image():
            with self._lock:
                frame = self._latest_frame
            if frame is None:
                # Return a small grey placeholder
                placeholder = np.full((240, 320, 3), 80, dtype=np.uint8)
                cv2.putText(placeholder, 'No image', (90, 125),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
                _, buf = cv2.imencode('.jpg', placeholder)
                frame = buf.tobytes()
            import io
            return send_file(io.BytesIO(frame), mimetype='image/jpeg')

        @app.route('/api/snapshots')
        def api_snapshots():
            with self._lock:
                return jsonify(self._snapshots)

        @app.route('/api/snapshots/<filename>')
        def serve_snapshot(filename):
            return send_from_directory(self.snapshot_dir, filename)

        @app.route('/tiles/<int:z>/<int:x>/<int:y>.jpg')
        def serve_tile(z, x, y):
            tile_dir = os.path.join(_STATIC_DIR, 'tiles', str(z), str(x))
            fname    = f'{y}.jpg'
            fpath    = os.path.join(tile_dir, fname)
            if os.path.exists(fpath):
                return send_from_directory(tile_dir, fname)
            # Tile not cached — return 404 (Leaflet will show grey)
            return '', 404

        @app.route('/api/snapshot', methods=['POST'])
        def manual_snapshot():
            """Manually save a snapshot via POST."""
            ts    = datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = f'manual_{ts}.jpg'
            fpath = os.path.join(self.snapshot_dir, fname)
            with self._lock:
                frame = self._latest_frame
            if frame:
                with open(fpath, 'wb') as f:
                    f.write(frame)
                snap = {'filename': fname, 'timestamp': ts, 'cluster_id': 'manual'}
                with self._lock:
                    self._snapshots.append(snap)
                return jsonify({'status': 'ok', 'filename': fname})
            return jsonify({'status': 'error', 'msg': 'no image'}), 400


def main(args=None):
    rclpy.init(args=args)
    node = WebVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
