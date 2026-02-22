# turret_sim_new_fast_v2.py
# v2: ODOM-DRIVEN turret simulation
# - No wall-clock timer drives physics/firing.
# - Uses PX4 VehicleOdometry.timestamp_sample (PX4 time) as the sole time base.
# - Updates bullets + firing schedule on every odometry message.
#
# Why v2:
# - With large PX4_SIM_SPEED_FACTOR, sim-time can advance rapidly while wall timers remain at 100 Hz.
# - Odom-driven updates keep bullet events aligned with sim-time progression and reduce "event lag"
#   where env advances steps before bullet state is published.
#
# Safety:
# - If odometry is not available, turret does nothing.
# - Clamps large time jumps to avoid runaway integration on pause/resume.
# - Uses sub-stepping for numerical stability.
#
# NOTE:
# - Timestamp units are assumed microseconds (PX4 convention).
#   If your timestamps are nanoseconds, change _TS_TO_SEC to 1e-9.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
import random


_TS_TO_SEC = 1e-6  # PX4 VehicleOdometry timestamps are typically in microseconds


class TurretSim(Node):
    def __init__(self):
        super().__init__('turret_sim_node_fast_v2')

        # --- Configuration ---
        self.turret_pos = np.array([100.0, 0.0, 0.0], dtype=np.float64)
        self.bullet_speed = 50.0

        # Fire interval in *sim seconds*
        self.fire_rate_min = 1.0
        self.fire_rate_max = 3.0
        self.next_fire_interval = random.uniform(self.fire_rate_min, self.fire_rate_max)

        self.bullet_range = 1550.0
        self.max_substep = 0.02  # seconds of sim time per integration substep
        self.max_dt_sim = 0.5    # clamp huge sim-time jumps

        # --- State ---
        self.bullets = []  # list of {'pos': np.ndarray(3), 'vel': np.ndarray(3)}
        self.drone_pos = np.zeros(3, dtype=np.float64)
        self.drone_vel = np.zeros(3, dtype=np.float64)

        self.have_odom = False
        self.odom_ts_us = None
        self.last_update_ts_us = None
        self.last_fire_ts_us = None

        self.is_armed = False

        # Track PX4 resets/time rewinds
        self._last_reset_counter = None

        # Marker publish throttling (avoid spamming RViz/CPU)
        self._marker_skip = 0
        self._marker_skip_N = 3  # publish markers every N odom msgs (tune as needed)

        # --- ROS2 comms ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        self.enable_sub = self.create_subscription(
            Bool, '/turret/enable', self.enable_callback, 10)

        self.bullets_pub = self.create_publisher(Float32MultiArray, '/turret/bullets', 10)
        self.marker_pub = self.create_publisher(Marker, '/turret/visual', 10)

    # -----------------------------
    # Callbacks
    # -----------------------------
    def enable_callback(self, msg: Bool):
        was_armed = self.is_armed
        self.is_armed = bool(msg.data)

        # On rising edge: reset firing schedule using current odom time if available
        if (not was_armed) and self.is_armed:
            self.next_fire_interval = random.uniform(self.fire_rate_min, self.fire_rate_max)
            if self.have_odom and self.odom_ts_us is not None:
                self.last_fire_ts_us = self.odom_ts_us

        if was_armed != self.is_armed:
            status = "🔫 ARMED" if self.is_armed else "🛑 DISARMED"
            self.get_logger().info(f"Turret Status: {status}")

    def odom_callback(self, msg: VehicleOdometry):
        # Use timestamp_sample if present, otherwise timestamp
        ts = int(getattr(msg, 'timestamp_sample', 0) or 0)
        if ts == 0:
            ts = int(getattr(msg, 'timestamp', 0) or 0)
        if ts <= 0:
            return

        # --- Reset-counter tracking (informational / time-anchor only) ---
        try:
            rc = int(getattr(msg, 'reset_counter', 0) or 0)
        except Exception:
            rc = None

        if rc is not None:
            if self._last_reset_counter is None:
                self._last_reset_counter = rc
            elif rc != self._last_reset_counter:
                # EKF / estimator resets can bump reset_counter without a full SITL reset.
                # Do NOT clear bullets here (that causes "group disappearance").
                self.get_logger().warn(
                    f"VehicleOdometry reset_counter changed ({self._last_reset_counter} -> {rc}). "
                    "Re-anchoring turret time base (bullets preserved).")
                self._last_reset_counter = rc
                # Re-anchor time to avoid negative dt on subsequent callbacks
                self.last_update_ts_us = ts
                self.last_fire_ts_us = ts
                self.odom_ts_us = ts
                # Apply kinematics after anchor reset (below) and continue.

        # Detect out-of-order delivery / time rewind
        # With BEST_EFFORT QoS, occasional out-of-order packets can happen.
        if self.last_update_ts_us is not None and ts < self.last_update_ts_us:
            back_us = self.last_update_ts_us - ts

            # Small backward jump: treat as out-of-order packet and ignore it completely.
            # (Prevents sudden bullet clearing / state regression.)
            BACKWARD_TOL_US = 50_000  # 50 ms
            if back_us <= BACKWARD_TOL_US:
                return

            # Large backward jump: treat as SITL reset / clock restart. Re-initialize anchors.
            RESET_BACKWARD_US = 1_000_000  # 1.0 s
            if back_us >= RESET_BACKWARD_US:
                self.get_logger().warn(
                    f"Odometry timestamp rewound by {back_us} us ({ts} < {self.last_update_ts_us}). "
                    "Re-initializing turret time anchors and clearing bullets.")
                self.odom_ts_us = ts
                self.last_update_ts_us = ts
                self.last_fire_ts_us = ts
                self.bullets = []
                self._publish_bullets()
                self._publish_markers_throttled()
                return

            # Medium backward jump: anchor time but keep bullets (safer than clearing).
            self.get_logger().warn(
                f"Odometry timestamp went backwards by {back_us} us ({ts} < {self.last_update_ts_us}). "
                "Re-anchoring time base (bullets preserved).")
            self.odom_ts_us = ts
            self.last_update_ts_us = ts
            self.last_fire_ts_us = ts
            return

        # Update drone kinematics AFTER timestamp sanity checks
        self.drone_pos = np.array([msg.position[0], msg.position[1], msg.position[2]], dtype=np.float64)
        self.drone_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]], dtype=np.float64)

        self.odom_ts_us = ts

        if not self.have_odom:
            # Initialize anchors on first valid odom
            self.have_odom = True
            self.last_update_ts_us = ts
            self.last_fire_ts_us = ts
            # Publish initial empty state (helps env sync)
            self._publish_bullets()
            self._publish_markers_throttled()
            return

        # Compute sim-time delta
        dt_sim = (ts - self.last_update_ts_us) * _TS_TO_SEC
        if dt_sim <= 0.0:
            return

        # Clamp large jumps (pause/resume, transport hiccups)
        if dt_sim > self.max_dt_sim:
            dt_sim = self.max_dt_sim

        # Fire schedule in sim-time
        if self.is_armed:
            if self.last_fire_ts_us is None:
                self.last_fire_ts_us = ts

            elapsed_fire = (ts - self.last_fire_ts_us) * _TS_TO_SEC
            if elapsed_fire >= self.next_fire_interval:
                self.fire_bullet()
                self.last_fire_ts_us = ts
                self.next_fire_interval = random.uniform(self.fire_rate_min, self.fire_rate_max)

        # Integrate bullets in sim-time with substeps
        remaining = dt_sim
        while remaining > 0.0:
            h = self.max_substep if remaining > self.max_substep else remaining
            self._integrate_bullets(h)
            remaining -= h

        self.last_update_ts_us = ts

        # Publish bullet list and markers
        self._publish_bullets()
        self._publish_markers_throttled()

    # -----------------------------
    # Bullet simulation
    # -----------------------------
    def _integrate_bullets(self, dt: float):
        active = []
        for b in self.bullets:
            b['pos'] = b['pos'] + (b['vel'] * dt)

            # Keep bullet "in the air" and within range
            if b['pos'][2] <= 0.0:
                if np.linalg.norm(b['pos'] - self.turret_pos) < self.bullet_range:
                    active.append(b)
        self.bullets = active

    def fire_bullet(self):
        # Predict lead using current drone state
        dist = np.linalg.norm(self.drone_pos - self.turret_pos)
        if dist < 0.1:
            dist = 0.1

        t_hit = dist / self.bullet_speed
        predicted = self.drone_pos + (self.drone_vel * t_hit)

        aim = predicted - self.turret_pos
        norm = np.linalg.norm(aim)
        if norm < 1e-6:
            return
        aim = aim / norm

        self.bullets.append({
            'pos': self.turret_pos.copy(),
            'vel': aim * self.bullet_speed
        })

        # Keep logging light (avoid slowing sim)
        # self.get_logger().info(f"Fire! Active Bullets: {len(self.bullets)}")

    # -----------------------------
    # Publishing
    # -----------------------------
    def _publish_bullets(self):
        flat = []
        for b in self.bullets:
            flat.extend([
                float(b['pos'][0]), float(b['pos'][1]), float(b['pos'][2]),
                float(b['vel'][0]), float(b['vel'][1]), float(b['vel'][2]),
            ])
        msg = Float32MultiArray()
        msg.data = flat
        self.bullets_pub.publish(msg)

    def _publish_markers_throttled(self):
        self._marker_skip = (self._marker_skip + 1) % self._marker_skip_N
        if self._marker_skip != 0:
            return
        self.publish_markers()

    def publish_markers(self):
        # Turret marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "turret_base"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.2

        # NED -> ENU for RViz
        marker.pose.position.x = float(self.turret_pos[1])
        marker.pose.position.y = float(self.turret_pos[0])
        marker.pose.position.z = float(-self.turret_pos[2])
        self.marker_pub.publish(marker)

        # Bullet points
        b_marker = Marker()
        b_marker.header.frame_id = "map"
        b_marker.ns = "bullets"
        b_marker.id = 1
        b_marker.type = Marker.POINTS
        b_marker.action = Marker.ADD
        b_marker.scale.x = 0.1
        b_marker.scale.y = 0.1
        b_marker.color.a = 1.0
        b_marker.color.r = 1.0
        b_marker.color.g = 0.0
        b_marker.color.b = 0.0

        for b in self.bullets:
            p = Point()
            p.x = float(b['pos'][1])
            p.y = float(b['pos'][0])
            p.z = float(-b['pos'][2])
            b_marker.points.append(p)

        self.marker_pub.publish(b_marker)


def main(args=None):
    rclpy.init(args=args)
    node = TurretSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()