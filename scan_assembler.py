#!/usr/bin/env python3
"""
scan_assembler.py  –  assemble 2D LD19 LaserScans captured during a
                      pan/tilt sweep (or nod) into a single 3D PointCloud2.

How it works
------------
The pan_tilt_node publishes:
  ~/scan_status   (std_msgs/String)         'started' / 'step:i:N' / 'complete' / 'aborted'
  ~/scan_capture  (sensor_msgs/JointState)  one msg per settled step, stamped

This node:
  1. On scan_status 'started'  -> clears the point buffer, begins accumulating.
  2. On each scan_capture      -> opens a short capture window; the next
                                  LaserScan whose stamp falls in that window is
                                  transformed into the target frame using TF
                                  (driven by /joint_states via
                                  robot_state_publisher) and appended.
  3. On scan_status 'complete' -> publishes the accumulated cloud as
                                  PointCloud2 on ~/cloud (latched) for RViz2.
     On 'aborted'              -> discards the partial cloud.

TF lookup strategy
------------------
This is a STOP-AND-CAPTURE scan: the platform is settled and stationary at
each step before its scan is captured. So we look up the transform at "latest
available" (Time()) rather than at the scan's exact stamp. During the dwell
the geometry isn't moving, so latest-TF == TF-at-scan-time, and this avoids
the "extrapolation into the future" race where the scan stamp is a few ms
ahead of the newest joint_states-driven TF.

Frames
------
  target_frame (param, default 'base_link')  – cloud is built in this frame
  the LaserScan's own frame ('base_laser')   – source frame, from /scan header

Subscribed:
  /scan                       (sensor_msgs/LaserScan)
  /pan_tilt_node/scan_status  (std_msgs/String)
  /pan_tilt_node/scan_capture (sensor_msgs/JointState)

Published:
  ~/cloud                     (sensor_msgs/PointCloud2)  latched (transient local)

Parameters:
  target_frame        (string, default 'base_link')
  scan_topic          (string, default '/scan')
  status_topic        (string, default '/pan_tilt_node/scan_status')
  capture_topic       (string, default '/pan_tilt_node/scan_capture')
  capture_window_sec  (float,  default 0.20)  how long after a capture trigger
                                              to accept one matching scan
  tf_timeout_sec      (float,  default 0.30)  TF lookup wait
  range_min           (float,  default 0.05)  drop returns closer than this (m)
  range_max           (float,  default 12.0)  drop returns farther than this (m)
"""

import math
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import LaserScan, PointCloud2, PointField, JointState
from std_msgs.msg import String, Header

import tf2_ros


def _build_pointcloud2(points, frame_id: str, stamp) -> PointCloud2:
    """
    points: list of (x, y, z) floats in `frame_id`.
    Returns an unorganized XYZ float32 PointCloud2.
    """
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.height = 1
    msg.width  = len(points)

    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step   = 12
    msg.row_step     = msg.point_step * msg.width
    msg.is_dense     = True

    buf = bytearray(msg.row_step)
    off = 0
    for (x, y, z) in points:
        struct.pack_into('<fff', buf, off, x, y, z)
        off += 12
    msg.data = bytes(buf)
    return msg


class ScanAssembler(Node):

    def __init__(self):
        super().__init__('scan_assembler')

        self.declare_parameter('target_frame',  'base_link')
        self.declare_parameter('scan_topic',    '/scan')
        self.declare_parameter('status_topic',  '/pan_tilt_node/scan_status')
        self.declare_parameter('capture_topic', '/pan_tilt_node/scan_capture')
        self.declare_parameter('capture_window_sec', 0.20)
        self.declare_parameter('tf_timeout_sec',     0.30)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 12.0)

        self._target   = self.get_parameter('target_frame').value
        scan_topic     = self.get_parameter('scan_topic').value
        status_topic   = self.get_parameter('status_topic').value
        capture_topic  = self.get_parameter('capture_topic').value
        self._win      = float(self.get_parameter('capture_window_sec').value)
        self._tf_to    = float(self.get_parameter('tf_timeout_sec').value)
        self._rmin     = float(self.get_parameter('range_min').value)
        self._rmax     = float(self.get_parameter('range_max').value)

        # TF
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # QoS
        qos_scan = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel  = QoSProfile(depth=10)
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # State
        self._lock         = threading.Lock()
        self._accumulating = False
        self._points       = []
        self._capture_open   = False
        self._capture_t0     = None   # rclpy.time.Time
        self._capture_t1     = None   # deadline
        self._capture_index  = -1

        # Pubs / subs
        self._cloud_pub = self.create_publisher(PointCloud2, '~/cloud', qos_latched)

        self.create_subscription(LaserScan,  scan_topic,    self._scan_cb,    qos_scan)
        self.create_subscription(String,     status_topic,  self._status_cb,  qos_rel)
        self.create_subscription(JointState, capture_topic, self._capture_cb, qos_rel)

        self.get_logger().info(
            f'scan_assembler ready  target={self._target}  '
            f'scan={scan_topic}  window={self._win}s  tf_timeout={self._tf_to}s')

    # ── status lifecycle ────────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        text = msg.data
        if text == 'started':
            with self._lock:
                self._accumulating = True
                self._points = []
                self._capture_open = False
            self.get_logger().info('scan started — accumulating')
        elif text == 'complete':
            self._finish(publish=True)
        elif text == 'aborted':
            self._finish(publish=False)
        # 'step:i:N' messages are informational; capture timing is driven
        # by the scan_capture topic instead.

    def _finish(self, publish: bool):
        with self._lock:
            self._accumulating = False
            self._capture_open = False
            n = len(self._points)
            pts = self._points
            self._points = []
        if publish and n > 0:
            cloud = _build_pointcloud2(pts, self._target, self.get_clock().now().to_msg())
            self._cloud_pub.publish(cloud)
            self.get_logger().info(f'scan complete — published cloud with {n} points')
        elif publish:
            self.get_logger().warn('scan complete — no points accumulated')
        else:
            self.get_logger().warn(f'scan aborted — discarded {n} points')

    # ── capture trigger ──────────────────────────────────────────────────────

    def _capture_cb(self, msg: JointState):
        if not self._accumulating:
            return
        t0 = Time.from_msg(msg.header.stamp)
        with self._lock:
            self._capture_open  = True
            self._capture_t0    = t0
            self._capture_t1    = t0 + Duration(seconds=self._win)
            self._capture_index = int(msg.velocity[0]) if len(msg.velocity) >= 1 else -1

    # ── scan consumption ───────────────────────────────────────────────────--

    def _scan_cb(self, scan: LaserScan):
        with self._lock:
            if not (self._accumulating and self._capture_open):
                return
            stamp = Time.from_msg(scan.header.stamp)
            # accept the first scan at/after the trigger and within the window
            if stamp < self._capture_t0:
                return
            if stamp > self._capture_t1:
                # window expired without a usable scan; close it (no capture)
                self._capture_open = False
                self.get_logger().warn(
                    f'capture {self._capture_index}: no scan within window')
                return
            # this scan qualifies — consume it and close the window
            self._capture_open = False
            idx = self._capture_index

        # transform outside the lock (TF + math can be slow-ish)
        added = self._append_scan(scan)
        self.get_logger().info(f'capture {idx}: +{added} pts')

    def _append_scan(self, scan: LaserScan) -> int:
        """
        Transform one LaserScan into target frame and append.
        Platform is settled/stationary at capture time, so we use the LATEST
        available transform (Time()) rather than the scan's exact stamp — this
        avoids the future-extrapolation race and is geometrically correct while
        the platform is not moving.
        """
        src_frame = scan.header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target, src_frame, Time(),
                timeout=Duration(seconds=self._tf_to))
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return 0

        # transform components
        t = tf.transform.translation
        q = tf.transform.rotation
        # rotation matrix from quaternion
        x, y, z, w = q.x, q.y, q.z, q.w
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        r00 = 1 - 2*(yy + zz); r01 = 2*(xy - wz);     r02 = 2*(xz + wy)
        r10 = 2*(xy + wz);     r11 = 1 - 2*(xx + zz); r12 = 2*(yz - wx)
        r20 = 2*(xz - wy);     r21 = 2*(yz + wx);     r22 = 1 - 2*(xx + yy)
        tx, ty, tz = t.x, t.y, t.z

        ang  = scan.angle_min
        inc  = scan.angle_increment
        out  = self._points
        rmin = max(self._rmin, scan.range_min)
        rmax = min(self._rmax, scan.range_max)
        added = 0

        for r in scan.ranges:
            a = ang
            ang += inc
            if r != r:            # NaN check (r != r is True for NaN)
                continue
            if math.isinf(r) or r < rmin or r > rmax:
                continue
            # point in laser frame (2D scan -> z=0 in its own frame)
            px = r * math.cos(a)
            py = r * math.sin(a)
            pz = 0.0
            # apply transform
            wxp = r00*px + r01*py + r02*pz + tx
            wyp = r10*px + r11*py + r12*pz + ty
            wzp = r20*px + r21*py + r22*pz + tz
            out.append((wxp, wyp, wzp))
            added += 1
        return added


def main(args=None):
    rclpy.init(args=args)
    node = ScanAssembler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
