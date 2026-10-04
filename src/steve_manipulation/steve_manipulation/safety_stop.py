"""
Safety stop shared by every script that moves Steve (steve_wbc controllers, manipulator).

Sources of a stop ("red"), in order of preference:
  em_stop  the robot's own EM-stop system: neo_relayboard's /emergency_stop_state
           (neo_msgs2/EmergencyStopState). Red while an EM stop is active or not yet released
           (buttons, software) or while the SICK S300 protective field is violated
           (scanner_stop). The relayboard no longer latches the EM stop on scanner_stop (see
           NeoRelayBoardNode.cpp, "MODIFIED (Steve)"), so the base driver keeps running on a
           scanner stop — these scripts stop base AND arm themselves while it is set.
           Required on the real robot; missing / stale state counts as red.
  lidar    fallback where no EM-stop system exists (MuJoCo, Gazebo): red while any LiDAR point
           is closer than d_safe (default 0.15 m) to the base footprint. Not used when the
           EM-stop system is running.
While red the caller stops base and arm; it resumes only after `clear_time` without red.

EmStopMonitor  ROS 2: /emergency_stop_state -> red / clear
LidarSafety    pure NumPy: points in base_link -> red / clear (no ROS needed, used by MuJoCo too)
ScanPoints     ROS 2: LaserScan topics -> points in base_link (TF looked up once per scan frame)
"""

import time

import numpy as np

# MMO-700 base footprint in base_link [m] (AABB of the base collision mesh MPO-700-BODY)
FOOTPRINT_MIN = (-0.325, -0.268)
FOOTPRINT_MAX = (0.305, 0.258)
D_SAFE = 0.15          # m  safety distance footprint <-> anything the LiDAR sees
CLEAR_TIME = 1.0       # s  field must stay clear this long before motion resumes
SCAN_MAX_AGE = 0.5     # s  older scans count as missing (red)
EM_STOP_TOPIC = "/emergency_stop_state"
EM_STOP_MAX_AGE = 1.0  # s  older EM-stop state counts as missing (red)
SELF_MARGIN = 0.02     # m  points inside the footprint (+ this) are the robot itself: ignored

# Real robot: laser_filters remove the robot's own body; Gazebo: raw scans
SCAN_TOPICS = {
    "real": ("/lidar_1/scan_filtered", "/lidar_2/scan_filtered"),
    "gazebo": ("/lidar_1/scan", "/lidar_2/scan"),
}


class EmStopMonitor:
    """The robot's EM-stop state (neo_relayboard). status() -> (red, reason)."""

    def __init__(self, node, topic=EM_STOP_TOPIC, max_age=EM_STOP_MAX_AGE):
        from neo_msgs2.msg import EmergencyStopState
        self.E, self.max_age = EmergencyStopState, max_age
        self.msg, self.t_msg = None, None
        node.create_subscription(EmergencyStopState, topic, self._cb, 10)

    def _cb(self, msg):
        self.msg, self.t_msg = msg, time.monotonic()

    @property
    def available(self):
        return self.msg is not None

    def status(self):
        m = self.msg
        if m is None:
            return True, "no EM-stop state (relayboard not running?)"
        if time.monotonic() - self.t_msg > self.max_age:
            return True, "EM-stop state stale (relayboard stopped?)"
        causes = [n for n, on in (("EM-stop button", m.emergency_button_stop),
                                  ("scanner protective field (red)", m.scanner_stop),
                                  ("software EM stop", m.software_stop)) if on]
        if m.emergency_state == self.E.EMSTOP:
            return True, "EM stop active" + (f": {', '.join(causes)}" if causes else "")
        if m.emergency_state == self.E.EMCONFIRMED:
            return True, "EM stop confirmed, waiting for release"
        if causes:
            return True, ", ".join(causes)
        return False, ""


class LidarSafety:
    """Fallback source (no EM-stop system): red while a LiDAR point is closer than d_safe to the
    base footprint. `points` is a callable returning Nx2 base_link points or None (no data)."""

    def __init__(self, points, d_safe=D_SAFE, footprint_min=FOOTPRINT_MIN, footprint_max=FOOTPRINT_MAX):
        self.points, self.d_safe = points, d_safe
        self.lo, self.hi = np.asarray(footprint_min, float), np.asarray(footprint_max, float)
        self.min_distance = np.inf  # over the run

    def distance(self, pts):
        """(distance footprint -> closest point, that point) for Nx2 base_link points."""
        pts = np.asarray(pts, float).reshape(-1, 2)
        inside = np.all((pts > self.lo - SELF_MARGIN) & (pts < self.hi + SELF_MARGIN), axis=1)
        pts = pts[~inside & np.all(np.isfinite(pts), axis=1)]
        if not len(pts):
            return np.inf, None
        d = np.hypot(*np.maximum(np.maximum(self.lo - pts, pts - self.hi), 0.0).T)
        i = int(np.argmin(d))
        return float(d[i]), pts[i]

    def status(self):
        pts = self.points()
        if pts is None:
            return True, "no LiDAR data"
        d, p = self.distance(pts)
        self.min_distance = min(self.min_distance, d)
        if d < self.d_safe:
            return True, f"LiDAR: object {d * 100:.0f} cm from the base at ({p[0]:+.2f}, {p[1]:+.2f}) m"
        return False, ""


class ClearTimer:
    """Stopped while red and for `clear_time` after the last red."""

    def __init__(self, clear_time=CLEAR_TIME):
        self.clear_time, self._last_red = clear_time, -np.inf

    def update(self, red, now):
        if red:
            self._last_red = now
        return red or now - self._last_red < self.clear_time


class ScanPoints:
    """Collects the latest LaserScan of each topic as points in `frame` (ROS 2 node given)."""

    def __init__(self, node, topics, frame="base_link", max_age=SCAN_MAX_AGE):
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener
        self.node, self.frame, self.max_age = node, frame, max_age
        self.tf = Buffer()
        self._tf_listener = TransformListener(self.tf, node)
        self._T = {}       # scan frame -> (R 2x2, t 2)
        self._scans = {}   # topic -> (points, receive time)
        self.topics = list(topics)
        for tp in self.topics:
            node.create_subscription(LaserScan, tp, lambda m, tp=tp: self._cb(tp, m), qos_profile_sensor_data)

    def _transform(self, frame_id):
        if frame_id not in self._T:
            from rclpy.time import Time
            tf = self.tf.lookup_transform(self.frame, frame_id, Time())
            t, q = tf.transform.translation, tf.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z)]])
            self._T[frame_id] = (R, np.array([t.x, t.y]))
        return self._T[frame_id]

    def _cb(self, topic, msg):
        try:
            R, t = self._transform(msg.header.frame_id)
        except Exception:
            return  # TF not available yet: the scan counts as missing
        r = np.asarray(msg.ranges, float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        pts = (R @ np.vstack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])).T + t
        self._scans[topic] = (pts, time.monotonic())

    def points(self):
        """All fresh points, or None (with the reason in self.missing) if a topic is stale."""
        now = time.monotonic()
        stale = [tp for tp in self.topics if tp not in self._scans or now - self._scans[tp][1] > self.max_age]
        self.missing = stale
        if stale:
            return None
        return np.vstack([self._scans[tp][0] for tp in self.topics])
