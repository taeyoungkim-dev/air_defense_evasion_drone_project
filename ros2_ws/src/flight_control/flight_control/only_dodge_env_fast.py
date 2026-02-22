import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from std_msgs.msg import Float32MultiArray, Bool
from std_srvs.srv import Empty


class OnlyDodgeEnv(gym.Env):
    """Only Dodge (Lazy Survivor) environment.

    FAST version: removes wall-clock sleeps in `step()` and replaces them with **VehicleOdometry timestamp-based**
    progression, so increasing PX4_SIM_SPEED_FACTOR actually speeds up RL stepping.

    Key idea:
      - One RL step advances the environment by `self.dt` **seconds of PX4 time** (VehicleOdometry.timestamp_sample).
      - That means wall time per step shrinks when the simulator is accelerated.

    NOTE:
      - If your turret/bullet spawner is driven by *wall time* (real-time), accelerating sim time will make
        fewer bullets happen per episode *in sim-time*. Fixing that requires making turret logic also use sim time.
    """

    def __init__(self):
        super().__init__()

        # 1) ROS2 node init
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('gym_env_node')

        # 2) QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 3) Pub/Sub
        self.pub_offboard_mode = self.node.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.pub_trajectory = self.node.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.pub_vehicle_command = self.node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )

        self.pub_turret_enable = self.node.create_publisher(Bool, '/turret/enable', 10)

        self.sub_odom = self.node.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile
        )
        self.sub_status = self.node.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos_profile
        )
        self.sub_bullets = self.node.create_subscription(
            Float32MultiArray, '/turret/bullets', self.bullets_cb, 10
        )

        self.reset_world_client = self.node.create_client(Empty, '/reset_world')

        # State
        self.nav_state = 0
        self.arming_state = 0

        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)

        # VehicleOdometry time tracking (PX4 time, microseconds)
        self.last_odom_ts_us = None

        # Home position
        self.home_pos = np.array([0.0, 0.0, -20.0])

        # Bullets
        self.raw_bullets = ()

        # Tunables
        self.action_scale_ratio = 5.0
        self.reset_retry_count = 0

        self.max_steps = 500
        self.steps = 0

        # RL step advances this much *PX4 time*
        self.dt = 0.1

        self.safe_radius = 10.0

        # PX4/Gazebo restart handles
        self.px4_process = None
        self.agent_process = None

        # If you restart PX4 from inside the env, you can also accelerate it.
        # (If you are starting PX4 externally, this won't affect it.)
        self.px4_sim_speed_factor = 1

        # Spin parameters (tuned for speed without pegging a core too hard)
        self._spin_poll_timeout = 0.0      # 0.0 = non-blocking
        self._spin_backoff_sleep = 0.0005  # small backoff to avoid 100% busy-spin

        # Gym spaces
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)

    # --------------------- Callbacks ---------------------
    def odom_cb(self, msg: VehicleOdometry):
        self.current_pos = np.array([msg.position[0], msg.position[1], msg.position[2]], dtype=np.float64)
        self.current_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]], dtype=np.float64)
        # PX4 timestamp is in microseconds (as shown in your `ros2 topic echo`)
        # Prefer timestamp_sample when available.
        ts = int(getattr(msg, 'timestamp_sample', 0) or 0)
        if ts <= 0:
            ts = int(getattr(msg, 'timestamp', 0) or 0)
        if ts > 0:
            self.last_odom_ts_us = ts

    def status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def bullets_cb(self, msg: Float32MultiArray):
        try:
            self.raw_bullets = tuple(msg.data)
        except Exception:
            self.raw_bullets = ()

    # --------------------- Timing helpers ---------------------
    def _spin_once_fast(self):
        """Process whatever is available right now."""
        rclpy.spin_once(self.node, timeout_sec=self._spin_poll_timeout)

    def _wait_for_odom(self, timeout_wall_s: float = 5.0) -> bool:
        """Wait until at least one VehicleOdometry arrives."""
        t0 = time.time()
        while self.last_odom_ts_us is None and (time.time() - t0) < timeout_wall_s:
            self._spin_once_fast()
            time.sleep(self._spin_backoff_sleep)
        return self.last_odom_ts_us is not None

    def _wait_odom_delta(self, delta_s: float, timeout_wall_s: float = 2.0) -> bool:
        """Wait until odom timestamp advances by delta_s (PX4 time), with a wall-time safety timeout."""
        if self.last_odom_ts_us is None:
            if not self._wait_for_odom(timeout_wall_s=timeout_wall_s):
                return False

        start_us = self.last_odom_ts_us
        target_us = start_us + int(delta_s * 1_000_000)
        t0 = time.time()

        while (self.last_odom_ts_us is None or self.last_odom_ts_us < target_us) and (time.time() - t0) < timeout_wall_s:
            self._spin_once_fast()
            time.sleep(self._spin_backoff_sleep)

        return self.last_odom_ts_us is not None and self.last_odom_ts_us >= target_us

    def _wait_sim_seconds(self, seconds: float, tick_s: float = 0.1, timeout_wall_s: float = 10.0) -> bool:
        """Wait `seconds` in PX4 time by waiting odom deltas in chunks (keeps publishing loops sane)."""
        remaining = float(seconds)
        t0 = time.time()
        while remaining > 1e-9:
            if (time.time() - t0) > timeout_wall_s:
                return False
            step = min(tick_s, remaining)
            ok = self._wait_odom_delta(step, timeout_wall_s=min(2.0, timeout_wall_s))
            if not ok:
                return False
            remaining -= step
        return True

    # --------------------- Turret control ---------------------
    def _set_turret_status(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        # publish multiple times (no wall sleeps in fast version)
        for _ in range(3):
            self.pub_turret_enable.publish(msg)
            self._spin_once_fast()

    # --------------------- Gym API ---------------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        print("\n>>> Resetting Environment (Only Dodge Mode, FAST)...")

        # stop turret first
        print(">>> 🛑 Turret DISARMED!")
        self._set_turret_status(False)

        if self.reset_retry_count >= 3:
            self.reset_retry_count = 0
            raise RuntimeError("⛔ Reset failed after 3 attempts! Manual intervention required.")

        self.steps = 0
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.raw_bullets = ()
        self.nav_state = 0
        self.arming_state = 0
        self.last_odom_ts_us = None

        # drain a few callbacks quickly
        for _ in range(20):
            self._spin_once_fast()

        # Ensure odom is flowing before we rely on sim-time waits
        if not self._wait_for_odom(timeout_wall_s=10.0):
            print(">>> ❌ No odometry received. Retrying reset...")
            self.reset_retry_count += 1
            return self.reset(seed=seed)

        # ---------------------------------------------------------
        # Offboard + arming: keep semantics, but replace wall waits with sim-time waits
        # ---------------------------------------------------------

        # Phase 1: heartbeat for ~2s sim-time
        print(">>> Phase 1: Sending Offboard heartbeat for 2 seconds (sim-time)...")
        # Maintain a ~10 Hz schedule in sim-time
        for _ in range(int(2.0 / 0.1)):
            self._publish_offboard_control_mode(mode="velocity")
            self._publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.0])
            if not self._wait_odom_delta(0.1, timeout_wall_s=2.0):
                break

        # Phase 2: switch to offboard
        print(">>> Phase 2: Switching to Offboard mode...")
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        # wait up to ~5s sim-time for nav_state==14
        for i in range(int(5.0 / 0.1)):
            self._publish_offboard_control_mode(mode="velocity")
            self._publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.0])
            self._wait_odom_delta(0.1, timeout_wall_s=2.0)
            if self.nav_state == 14:
                print(f">>> Offboard mode activated! (Attempt: {i})")
                break

        if self.nav_state != 14:
            print(">>> Offboard mode switch failed. Retrying...")
            self.reset_retry_count += 1
            return self.reset(seed=seed)

        # Phase 3: arm
        print(">>> Phase 3: Arming...")
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        for i in range(int(5.0 / 0.1)):
            self._publish_offboard_control_mode(mode="velocity")
            self._publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.0])
            self._wait_odom_delta(0.1, timeout_wall_s=2.0)
            if self.arming_state == 2:
                print(f">>> Armed successfully! (Attempt: {i})")
                break

        if self.nav_state != 14 or self.arming_state != 2:
            print(f">>> Arming Failed - Nav: {self.nav_state}, Arm: {self.arming_state}. Retrying...")
            self.reset_retry_count += 1
            return self.reset(seed=seed)

        print(">>> Returning to Home (Position Control)...")
        home_target = self.home_pos.tolist()

        # RTH timeout: 30s sim-time
        rth_deadline_ok = True
        rth_start_us = self.last_odom_ts_us
        rth_timeout_us = int(30.0 * 1_000_000)

        while True:
            self._publish_offboard_control_mode(mode="position")
            self._publish_trajectory_setpoint(position=home_target, yaw=0.0)

            # allow state updates
            self._wait_odom_delta(0.1, timeout_wall_s=2.0)

            dist_to_home = float(np.linalg.norm(self.current_pos - np.array(home_target)))
            if dist_to_home < 0.5:
                print(">>> Arrived at Home! Starting Lazy Survival Mode...")

                # stabilize 2s sim-time while holding position
                print(">>> Stabilizing for 2 seconds (sim-time)...")
                stabilize_us = self.last_odom_ts_us
                while self.last_odom_ts_us is not None and (self.last_odom_ts_us - stabilize_us) < int(2.0 * 1_000_000):
                    self._publish_offboard_control_mode(mode="position")
                    self._publish_trajectory_setpoint(position=home_target, yaw=0.0)
                    if not self._wait_odom_delta(0.1, timeout_wall_s=2.0):
                        break

                print(">>> 🔫 Turret ARMED! Combat Start!")
                self._set_turret_status(True)
                break

            # Sim-time RTH timeout check
            if self.last_odom_ts_us is not None and (self.last_odom_ts_us - rth_start_us) > rth_timeout_us:
                rth_deadline_ok = False

            if not rth_deadline_ok:
                print(">>> 🚨 Critical Failure: RTH Timeout (sim-time)! Drone is likely flipped.")
                print(">>> Initiating PX4/Gazebo full restart...")

                reset_success = self._reset_gazebo_world()
                if not reset_success:
                    self.reset_retry_count += 1
                    if self.reset_retry_count >= 3:
                        raise RuntimeError("⛔ PX4/Gazebo restart failed after 3 attempts!")
                    print(f">>> Retrying... (Attempt {self.reset_retry_count}/3)")
                    return self.reset(seed=seed)

                self.reset_retry_count = 0
                print(">>> ✅ Restart successful! Starting fresh episode...")
                return self.reset(seed=seed)

        print(">>> Ready to Survive! Lazy Dodge Mode ON.\n")
        self.reset_retry_count = 0
        return self._get_obs(), {}

    def step(self, action):
        """One RL step = advance `dt` seconds in PX4 time (VehicleOdometry timestamp)."""
        self.steps += 1

        # Publish control
        self._publish_offboard_control_mode(mode="velocity")
        real_action = action * self.action_scale_ratio
        self._publish_trajectory_setpoint(velocity=real_action)

        # Advance simulation by dt (PX4 time) instead of wall-time sleeping
        ok = self._wait_odom_delta(self.dt, timeout_wall_s=2.0)
        if not ok:
            # Fail-safe: if odom stalled, don't hang forever
            for _ in range(50):
                self._spin_once_fast()
            # We still proceed with whatever state we have.

        # Process bullets
        closest_bullet_vec, min_dist, is_hit = self._process_bullets()

        # Observation: home relative + self vel + closest bullet vec
        home_rel = self.home_pos - self.current_pos
        obs = np.concatenate([home_rel, self.current_vel, closest_bullet_vec]).astype(np.float32)

        # Reward / termination
        reward = 0.0
        done = False
        truncated = False

        dist_to_home = float(np.linalg.norm(home_rel))

        # survival reward
        reward += 1.0

        # energy penalty
        reward += -0.05 * float(np.linalg.norm(action))

        # center distance penalty (quadratic)
        reward -= 0.005 * (dist_to_home ** 2)

        # near-miss bonus
        if (not is_hit) and (min_dist < 2.0):
            reward += 1.0

        # termination
        if is_hit:
            reward -= 500.0
            done = True
            print(f"💥 Hit! Survived {self.steps} steps. Home dist: {dist_to_home:.2f}m")

        elif self.current_pos[2] > -0.2:
            reward -= 500.0
            done = True
            print(f"💥 Crashed! Survived {self.steps} steps.")

        elif dist_to_home > self.safe_radius:
            reward -= 500.0
            done = True
            print(f"🚨 Desertion! Too far from home: {dist_to_home:.2f}m (limit: {self.safe_radius}m)")

        elif self.steps >= self.max_steps:
            done = True
            truncated = True
            print(f"⏱️ Time's up! Successfully survived {self.max_steps} steps!")

        return obs, reward, done, truncated, {}

    # --------------------- Observations / bullets ---------------------
    def _get_obs(self):
        home_rel = self.home_pos - self.current_pos
        dummy_bullet = np.array([100.0, 100.0, 100.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return np.concatenate([home_rel, self.current_vel, dummy_bullet]).astype(np.float32)

    def _process_bullets(self):
        """CCD hit test for all bullets + choose closest approaching bullet for observation."""
        if len(self.raw_bullets) == 0 or (len(self.raw_bullets) % 6) != 0:
            return np.array([100.0, 100.0, 100.0, 0.0, 0.0, 0.0], dtype=np.float64), 999.0, False

        drone_p = self.current_pos

        closest_dist = 999.0
        closest_bullet_vec = np.zeros(6, dtype=np.float64)
        hit_detected = False

        threat_candidates = []

        bullets_array = np.array(self.raw_bullets, dtype=np.float64).reshape(-1, 6)

        for bullet_data in bullets_array:
            b_curr = bullet_data[:3]
            b_vel = bullet_data[3:]

            # CCD hit test
            speed = float(np.linalg.norm(b_vel))
            if speed < 0.1:
                dist_segment = float(np.linalg.norm(drone_p - b_curr))
            else:
                b_prev = b_curr - (b_vel * self.dt)
                dist_segment = float(self._point_line_segment_distance(drone_p, b_prev, b_curr))

            if dist_segment < 0.5:
                hit_detected = True

            # approaching?
            vec_b_to_d = drone_p - b_curr
            dot_prod = float(np.dot(b_vel, vec_b_to_d))
            if dot_prod > 0:
                dist_curr = float(np.linalg.norm(vec_b_to_d))
                threat_candidates.append((dist_curr, b_curr, b_vel))

        if threat_candidates:
            threat_candidates.sort(key=lambda x: x[0])
            best = threat_candidates[0]
            rel_pos = best[1] - drone_p
            closest_bullet_vec = np.concatenate([rel_pos, best[2]])
            closest_dist = best[0]
        else:
            closest_bullet_vec = np.array([100.0, 100.0, 100.0, 0.0, 0.0, 0.0], dtype=np.float64)
            closest_dist = 999.0

        return closest_bullet_vec, closest_dist, hit_detected

    @staticmethod
    def _point_line_segment_distance(point, start, end):
        line_vec = end - start
        point_vec = point - start
        line_len_sq = float(np.dot(line_vec, line_vec))

        if line_len_sq == 0.0:
            return float(np.linalg.norm(point_vec))

        t = float(np.dot(point_vec, line_vec) / line_len_sq)
        t = max(0.0, min(1.0, t))
        nearest = start + t * line_vec
        return float(np.linalg.norm(point - nearest))

    # --------------------- PX4 messaging ---------------------
    def _publish_offboard_control_mode(self, mode="velocity"):
        msg = OffboardControlMode()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)

        if mode == "position":
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
        else:
            msg.position = False
            msg.velocity = True
            msg.acceleration = False

        self.pub_offboard_mode.publish(msg)

    def _publish_trajectory_setpoint(self, position=None, velocity=None, yaw=float('nan')):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.yaw = float(yaw)

        if position is not None:
            msg.position = [float(position[0]), float(position[1]), float(position[2])]
            msg.velocity = [float('nan')] * 3
        elif velocity is not None:
            msg.position = [float('nan')] * 3
            msg.velocity = [float(velocity[0]), float(velocity[1]), float(velocity[2])]
        else:
            msg.position = [float('nan')] * 3
            msg.velocity = [float('nan')] * 3

        self.pub_trajectory.publish(msg)

    def _publish_vehicle_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_command.publish(msg)

    # --------------------- Full restart (kept from original) ---------------------
    def _reset_gazebo_world(self):
        import subprocess
        import os

        print("\n>>> 🔄 PX4/Gazebo 완전 재시작 시작...")

        if not self._kill_px4_gazebo():
            print(">>> ⚠️ 프로세스 종료 실패, 계속 진행...")

        if not self._start_micro_xrce_agent():
            print(">>> ⚠️ Agent 재시작 실패, 계속 진행...")

        if not self._start_px4_gazebo():
            print(">>> ❌ PX4/Gazebo 재시작 실패!")
            return False

        if not self._wait_for_ros2_connection(timeout=30.0):
            print(">>> ❌ ROS2 연결 실패!")
            return False

        print(">>> ✅ PX4/Gazebo 재시작 완료! 드론이 원점에 있습니다.")
        return True

    def _kill_px4_gazebo(self):
        import subprocess

        print(">>> 🔴 Killing PX4 and Gazebo processes...")
        processes_to_kill = ['px4', 'gzserver', 'gzclient', 'gazebo', 'MicroXRCEAgent']

        for proc_name in processes_to_kill:
            try:
                subprocess.run(f"pkill -9 {proc_name}", shell=True, capture_output=True, timeout=2.0)
            except Exception:
                pass

        # keep small wall wait to let OS reap processes (rarely the bottleneck)
        time.sleep(1.0)
        print(">>> ✅ All processes killed!")
        return True

    def _start_micro_xrce_agent(self):
        import subprocess

        print(">>> 🌉 Starting Micro-XRCE-DDS-Agent...")
        try:
            self.agent_process = subprocess.Popen(
                ["MicroXRCEAgent", "udp4", "-p", "8888"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            print(f">>> Agent started (PID: {self.agent_process.pid})")
            time.sleep(1.0)

            return self.agent_process.poll() is None

        except Exception as e:
            print(f">>> ⚠️ Agent start failed: {e}")
            return True

    def _start_px4_gazebo(self):
        import subprocess
        import os

        print(">>> 🚀 Starting PX4 SITL + Gazebo...")
        px4_path = os.path.expanduser("~/PX4-Autopilot")
        if not os.path.exists(px4_path):
            print(f">>> ❌ PX4-Autopilot not found at: {px4_path}")
            return False

        try:
            cmd = (
                f"cd {px4_path} && "
                f"HEADLESS=1 PX4_SIM_SPEED_FACTOR={int(self.px4_sim_speed_factor)} "
                f"make px4_sitl gazebo-classic_iris"
            )

            self.px4_process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            print(f">>> PX4 started (PID: {self.px4_process.pid})")
            # allow some wall time for SITL boot
            time.sleep(10.0)

            if self.px4_process.poll() is None:
                print(">>> ✅ PX4/Gazebo successfully started!")
                return True

            print(">>> ❌ PX4/Gazebo failed to start!")
            return False

        except Exception as e:
            print(f">>> ❌ Failed to start PX4: {e}")
            return False

    def _wait_for_ros2_connection(self, timeout=30.0):
        print(">>> ⏳ Waiting for ROS2 connection...")

        start_time = time.time()
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.nav_state = 0
        self.arming_state = 0
        self.last_odom_ts_us = None

        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if self.last_odom_ts_us is not None and np.any(np.abs(self.current_pos) > 0.001):
                print(">>> ✅ ROS2 connection established!")
                print(
                    f">>> 현재 위치: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f})"
                )
                return True

        print(">>> ❌ ROS2 connection timeout!")
        return False
