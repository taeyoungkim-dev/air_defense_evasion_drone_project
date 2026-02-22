# turret_sim_new_fast.py
# Safe variant of turret_sim_new.py:
# - Uses PX4 VehicleOdometry.timestamp_sample (PX4 time) as the time base for BOTH firing schedule and bullet physics.
# - This makes bullet density consistent in *sim time* and allows PX4_SIM_SPEED_FACTOR acceleration to work end-to-end.
#
# Notes:
# - If odometry is not available yet, this node will not fire/update bullets (safety-first).
# - Timestamp units are assumed to be microseconds (PX4 convention). If your stack is different, adjust _TS_TO_SEC.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
import random
import time as _time  # only for fallback logging timestamps / not for sim timing


_TS_TO_SEC = 1e-6  # PX4 VehicleOdometry timestamps are typically in microseconds


class TurretSim(Node):
    def __init__(self):
        super().__init__('turret_sim_node')

        # --- [1. Turret configuration] ---
        self.turret_pos = np.array([100.0, 0.0, 0.0])
        self.bullet_speed = 50.0

        # Random fire interval, in *sim seconds*
        self.fire_rate_min = 1.0
        self.fire_rate_max = 3.0
        self.next_fire_interval = 2.0

        self.bullet_range = 1550.0

        # Physics integration: limit substep to keep stability when sim advances fast
        self.max_substep = 0.02  # seconds of sim time per integration substep

        # --- [2. State] ---
        self.bullets = []  # [{'pos': vec3, 'vel': vec3}, ...]
        self.drone_pos = np.zeros(3)
        self.drone_vel = np.zeros(3)

        # Time base from odometry (PX4 time)
        self.have_odom = False
        self.odom_ts_us = None  # latest timestamp_sample in microseconds
        self.last_update_ts_us = None
        self.last_fire_ts_us = None

        # Arm switch (default False for safety)
        self.is_armed = False

        # --- [3. ROS2 comms] ---
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

        # Timer: just a driver to run update_callback frequently.
        # We do NOT use wall time for physics; we integrate by odom timestamp deltas.
        self.timer = self.create_timer(0.01, self.update_callback)

    def odom_callback(self, msg: VehicleOdometry):
        self.drone_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.drone_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])

        # Prefer timestamp_sample (sample time). Fall back to timestamp.
        ts = int(getattr(msg, 'timestamp_sample', 0) or 0)
        if ts == 0:
            ts = int(getattr(msg, 'timestamp', 0) or 0)

        if ts > 0:
            self.odom_ts_us = ts
            if not self.have_odom:
                # Initialize time anchors on first odom
                self.have_odom = True
                self.last_update_ts_us = ts
                self.last_fire_ts_us = ts

    def enable_callback(self, msg: Bool):
        was_armed = self.is_armed
        self.is_armed = bool(msg.data)

        # On rising edge: reset firing schedule in sim-time
        if (not was_armed) and self.is_armed:
            self.next_fire_interval = random.uniform(self.fire_rate_min, self.fire_rate_max)
            if self.have_odom and self.odom_ts_us is not None:
                self.last_fire_ts_us = self.odom_ts_us

        if was_armed != self.is_armed:
            status = "🔫 ARMED" if self.is_armed else "🛑 DISARMED"
            self.get_logger().info(f"Turret Status: {status}")

    def update_callback(self):
        # Safety: do nothing until we have odometry time
        if not self.have_odom or self.odom_ts_us is None:
            return

        now_us = self.odom_ts_us

        # -----------------------------
        # 0) Compute sim-time delta for physics
        # -----------------------------
        if self.last_update_ts_us is None:
            self.last_update_ts_us = now_us
            return

        dt_sim = (now_us - self.last_update_ts_us) * _TS_TO_SEC
        if dt_sim <= 0.0:
            return

        # Guard against huge jumps (e.g., pause/resume). Clamp to a reasonable max step.
        if dt_sim > 0.5:
            dt_sim = 0.5

        # -----------------------------
        # 1) Fire logic in sim-time
        # -----------------------------
        if self.is_armed:
            if self.last_fire_ts_us is None:
                self.last_fire_ts_us = now_us

            elapsed_fire = (now_us - self.last_fire_ts_us) * _TS_TO_SEC
            if elapsed_fire >= self.next_fire_interval:
                self.fire_bullet()
                self.last_fire_ts_us = now_us
                self.next_fire_interval = random.uniform(self.fire_rate_min, self.fire_rate_max)

        # -----------------------------
        # 2) Physics integration in sim-time with substeps
        # -----------------------------
        remaining = dt_sim
        while remaining > 0.0:
            h = self.max_substep if remaining > self.max_substep else remaining
            self._integrate_bullets(h)
            remaining -= h

        self.last_update_ts_us = now_us

        # -----------------------------
        # 3) Publish bullet data + markers
        # -----------------------------
        self._publish_bullets()
        self.publish_markers()

    def _integrate_bullets(self, dt: float):
        active_bullets = []
        for b in self.bullets:
            b['pos'] += b['vel'] * dt

            # Keep only bullets that are still "in the air" and within range
            if b['pos'][2] <= 0.0:
                dist = np.linalg.norm(b['pos'] - self.turret_pos)
                if dist < self.bullet_range:
                    active_bullets.append(b)
        self.bullets = active_bullets

    def _publish_bullets(self):
        flat_list = []
        for b in self.bullets:
            flat_list.extend([
                float(b['pos'][0]), float(b['pos'][1]), float(b['pos'][2]),
                float(b['vel'][0]), float(b['vel'][1]), float(b['vel'][2]),
            ])
        msg = Float32MultiArray()
        msg.data = flat_list
        self.bullets_pub.publish(msg)

    def fire_bullet(self):
        dist_to_drone = np.linalg.norm(self.drone_pos - self.turret_pos)
        if dist_to_drone < 0.1:
            dist_to_drone = 0.1

        time_to_hit = dist_to_drone / self.bullet_speed
        predicted_pos = self.drone_pos + (self.drone_vel * time_to_hit)

        aim_vec = predicted_pos - self.turret_pos
        norm = np.linalg.norm(aim_vec)
        if norm < 1e-6:
            return
        aim_vec = aim_vec / norm

        new_bullet = {
            'pos': self.turret_pos.copy(),
            'vel': aim_vec * self.bullet_speed
        }
        self.bullets.append(new_bullet)
        self.get_logger().info(f"Fire! Active Bullets: {len(self.bullets)}")

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

        # Bullet markers
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
