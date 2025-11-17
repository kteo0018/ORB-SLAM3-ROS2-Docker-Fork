#!/usr/bin/env python3
import os
import threading
import math
from datetime import datetime
from typing import Optional, TextIO, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger


def _stamp_to_float_sec(sec: int, nanosec: int) -> float:
    return float(sec) + float(nanosec) * 1e-9


class Traj1PoseGpsLogger(Node):
    def __init__(self) -> None:
        super().__init__('traj1_pose_gps_logger')

        # Parameters for topic names
        self.declare_parameter('pose_topic', '/robot/robot_pose_slam')
        self.declare_parameter('odometry_topic', '/model/x500_custom_0/odometry_with_covariance')
        self.declare_parameter('gps_topic', '/world/z_simple_palm_plantation/model/x500_custom_0/link/gps_link/sensor/gps/navsat')

        pose_topic: str = self.get_parameter('pose_topic').get_parameter_value().string_value
        odometry_topic: str = self.get_parameter('odometry_topic').get_parameter_value().string_value
        gps_topic: str = self.get_parameter('gps_topic').get_parameter_value().string_value

        # Output directory (timestamped by default)
        timestamp_str = datetime.now().strftime('%y%m%d_%H%M')
        default_out_dir = os.path.join('./colcon_ws', f'traj1_logs_{timestamp_str}')
        self.declare_parameter('output_dir', default_out_dir)
        self._output_dir: str = self.get_parameter('output_dir').get_parameter_value().string_value

        # Optional GPS log in TUM pose columns after converting to local ENU
        self.declare_parameter('write_gps_tum', True)
        self._write_gps_tum: bool = bool(self.get_parameter('write_gps_tum').get_parameter_value().bool_value)
        # GPS reference for ENU; if not provided, will auto-initialize from first GPS message
        self.declare_parameter('gps_ref_lat', float('nan'))
        self.declare_parameter('gps_ref_lon', float('nan'))
        self.declare_parameter('gps_ref_alt', float('nan'))
        self._gps_ref: Optional[Tuple[float, float, float]] = None
        try:
            lat0 = float(self.get_parameter('gps_ref_lat').get_parameter_value().double_value)
            lon0 = float(self.get_parameter('gps_ref_lon').get_parameter_value().double_value)
            alt0 = float(self.get_parameter('gps_ref_alt').get_parameter_value().double_value)
            if lat0 == lat0 and lon0 == lon0 and alt0 == alt0:  # not NaN
                self._gps_ref = (lat0, lon0, alt0)
        except Exception:
            self._gps_ref = None

        # QoS suitable for sensor streams
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        # Writers
        self._lock = threading.Lock()
        self._paused: bool = False
        self._pose_file: Optional[TextIO] = None
        self._odom_file: Optional[TextIO] = None
        self._gps_file: Optional[TextIO] = None

        # Ensure directory exists
        os.makedirs(self._output_dir, exist_ok=True)

        # Subscriptions
        self._pose_sub = self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos)
        self._odom_sub = self.create_subscription(Odometry, odometry_topic, self._on_odom, qos)
        self._gps_sub = self.create_subscription(NavSatFix, gps_topic, self._on_gps, qos)

        # Command and services
        self._cmd_sub = self.create_subscription(StringMsg, 'traj1/command', self._on_command, 10)
        self._srv_pause = self.create_service(Trigger, 'traj1/pause', self._on_srv_pause)
        self._srv_resume = self.create_service(Trigger, 'traj1/resume', self._on_srv_resume)
        self._srv_flush = self.create_service(Trigger, 'traj1/flush', self._on_srv_flush)

        self.get_logger().info(
            f"Logging to '{self._output_dir}' (pose: {pose_topic}, odom: {odometry_topic}, gps: {gps_topic})"
        )

    # --- Handlers ---
    def _on_pose(self, msg: PoseStamped) -> None:
        if self._paused:
            return
        t = _stamp_to_float_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        p = msg.pose.position
        q = msg.pose.orientation
        line = f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"
        with self._lock:
            if self._pose_file is None:
                self._pose_file = open(os.path.join(self._output_dir, 'orb_slam3.tum'), 'a', buffering=1)
            self._pose_file.write(line)

    def _on_odom(self, msg: Odometry) -> None:
        if self._paused:
            return
        t = _stamp_to_float_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        line = f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"
        with self._lock:
            if self._odom_file is None:
                self._odom_file = open(os.path.join(self._output_dir, 'ground_truth.tum'), 'a', buffering=1)
            self._odom_file.write(line)

    def _on_gps(self, msg: NavSatFix) -> None:
        if self._paused or not self._write_gps_tum:
            return
        # Convert geodetic to local ENU coordinates relative to reference
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        alt = float(msg.altitude)
        if self._gps_ref is None:
            self._gps_ref = (lat, lon, alt)
            self.get_logger().info(f"Initialized GPS ENU reference: lat={lat:.8f}, lon={lon:.8f}, alt={alt:.3f}")
        x, y, z = self._geodetic_to_enu(lat, lon, alt, *self._gps_ref)
        # Use message header timestamp if available
        t = _stamp_to_float_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        line = f"{t:.9f} {x:.6f} {y:.6f} {z:.6f} 0.000000 0.000000 0.000000 1.000000\n"
        with self._lock:
            if self._gps_file is None:
                self._gps_file = open(os.path.join(self._output_dir, 'gps_navsat.tum'), 'a', buffering=1)
            self._gps_file.write(line)

    def _on_command(self, msg: StringMsg) -> None:
        cmd = (msg.data or '').strip().lower()
        if cmd in ('pause', 'resume', 'toggle'):
            if cmd == 'toggle':
                self._paused = not self._paused
            elif cmd == 'pause':
                self._paused = True
            else:
                self._paused = False
            state = 'paused' if self._paused else 'resumed'
            self.get_logger().info(f"Logging {state} via command topic")
        elif cmd == 'flush':
            self._flush_files()
            self.get_logger().info('Flushed trajectory files')

    # --- Services ---
    def _on_srv_pause(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._paused = True
        res.success = True
        res.message = 'Logging paused'
        return res

    def _on_srv_resume(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._paused = False
        res.success = True
        res.message = 'Logging resumed'
        return res

    def _on_srv_flush(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._flush_files()
        res.success = True
        res.message = 'Flushed'
        return res

    # --- Utils ---
    def _flush_files(self) -> None:
        with self._lock:
            for f in (self._pose_file, self._odom_file, self._gps_file):
                try:
                    if f is not None:
                        f.flush()
                        os.fsync(f.fileno())
                except Exception:
                    pass

    # --- GPS conversion helpers (WGS84 -> ECEF -> ENU) ---
    @staticmethod
    def _wgs84_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
        # WGS84 ellipsoid constants
        a = 6378137.0
        e2 = 6.69437999014e-3
        lat = lat_deg * 3.141592653589793 / 180.0
        lon = lon_deg * 3.141592653589793 / 180.0
        sin_lat = math.sin(lat)
        cos_lat = math.cos(lat)
        sin_lon = math.sin(lon)
        cos_lon = math.cos(lon)
        N = a / (1.0 - e2 * sin_lat * sin_lat) ** 0.5
        x = (N + alt_m) * cos_lat * cos_lon
        y = (N + alt_m) * cos_lat * sin_lon
        z = (N * (1.0 - e2) + alt_m) * sin_lat
        return x, y, z

    @staticmethod
    def _ecef_to_enu(x: float, y: float, z: float, lat0_deg: float, lon0_deg: float, alt0_m: float) -> Tuple[float, float, float]:
        x0, y0, z0 = Traj1PoseGpsLogger._wgs84_to_ecef(lat0_deg, lon0_deg, alt0_m)
        dx = x - x0
        dy = y - y0
        dz = z - z0
        lat0 = lat0_deg * 3.141592653589793 / 180.0
        lon0 = lon0_deg * 3.141592653589793 / 180.0
        sin_lat = math.sin(lat0)
        cos_lat = math.cos(lat0)
        sin_lon = math.sin(lon0)
        cos_lon = math.cos(lon0)
        t = -sin_lon * dx + cos_lon * dy
        e = t
        n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return e, n, u

    @staticmethod
    def _geodetic_to_enu(lat_deg: float, lon_deg: float, alt_m: float, lat0_deg: float, lon0_deg: float, alt0_m: float) -> Tuple[float, float, float]:
        x, y, z = Traj1PoseGpsLogger._wgs84_to_ecef(lat_deg, lon_deg, alt_m)
        return Traj1PoseGpsLogger._ecef_to_enu(x, y, z, lat0_deg, lon0_deg, alt0_m)

    def destroy_node(self) -> None:
        try:
            self._flush_files()
        finally:
            with self._lock:
                for attr in ('_pose_file', '_odom_file', '_gps_file'):
                    f = getattr(self, attr, None)
                    try:
                        if f is not None:
                            f.close()
                    except Exception:
                        pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Traj1PoseGpsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


