import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from std_msgs.msg import Float32MultiArray, Bool  # Bool 추가
from std_srvs.srv import Empty

class OnlyDodgeEnv(gym.Env):
    """
    Lazy Survivor Environment
    
    목표: 최소한의 움직임으로 홈 포지션 근처에서 총알을 회피하며 생존
    
    설계 철학:
    - Survival First: 살아남는 것이 유일한 목표
    - Laziness: 불필요한 움직임은 에너지 소비 (보상 감점)
    - Home-Loving: 홈 포지션 근처 안전 반경 내에 머무름
    """
    
    def __init__(self):
        super(OnlyDodgeEnv, self).__init__()
        
        # 1. ROS2 노드 초기화
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('gym_env_node')

        # 2. QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 3. Pub/Sub 설정
        self.pub_offboard_mode = self.node.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.pub_trajectory = self.node.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.pub_vehicle_command = self.node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # [추가] 터렛 제어용 퍼블리셔
        self.pub_turret_enable = self.node.create_publisher(Bool, '/turret/enable', 10)

        self.sub_odom = self.node.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)
        self.sub_status = self.node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos_profile)
        
        # 터렛 총알 데이터 수신
        self.sub_bullets = self.node.create_subscription(Float32MultiArray, '/turret/bullets', self.bullets_cb, 10)
        
        # Gazebo 리셋 서비스 클라이언트
        self.reset_world_client = self.node.create_client(Empty, '/reset_world')

        self.nav_state = 0
        self.arming_state = 0

        # 4. 변수 초기화
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        
        # 홈 포지션 (목표 지점 제거, 홈에서 생존이 목표)
        self.home_pos = np.array([0.0, 0.0, -20.0])  # 20m 높이
        self.raw_bullets = ()  # tuple로 초기화 (타입 일관성)

        #CHANGEABLE
        self.action_scale_ratio = 5.0
        # 리셋 재시도 카운터 (무한 루프 방지)
        self.reset_retry_count = 0 

        # 최대 스텝 (30초)
        #CHANGEABLE
        self.max_steps = 500
        self.steps = 0
        self.dt = 0.1
        
        # 안전 반경 (Deadzone)
        #CHANGEABLE
        self.safe_radius = 10.0  # meters
        
        # PX4/Gazebo 프로세스 핸들 (재시작용)
        self.px4_process = None
        self.agent_process = None

        # 5. Gym Space 정의
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        # Obs: [Home_Rel(3), Self_Vel(3), Bullet_Rel(3), Bullet_Vel(3)]
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)

    # --- Callbacks ---
    def odom_cb(self, msg):
        self.current_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.current_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])

    def status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def bullets_cb(self, msg):
        try:
            self.raw_bullets = tuple(msg.data)
        except Exception:
            self.raw_bullets = ()
    
    def _set_turret_status(self, enabled: bool):
        """터렛 사격 허가 제어"""
        msg = Bool()
        msg.data = enabled
        self.pub_turret_enable.publish(msg)
        
        # 확실하게 전달되도록 여러 번 발행
        for _ in range(3):
            self.pub_turret_enable.publish(msg)
            time.sleep(0.01)

    # --- Gym Methods ---
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        print("\n>>> Resetting Environment (Only Dodge Mode)...")
        
        # [1] 리셋 시작하자마자 사격 중지!
        print(">>> 🛑 Turret DISARMED!")
        self._set_turret_status(False)
        
        # 무한 재시도 방지
        if self.reset_retry_count >= 3:
            self.reset_retry_count = 0  # 리셋
            raise RuntimeError("⛔ Reset failed after 3 attempts! Manual intervention required.")
        
        self.steps = 0
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.raw_bullets = ()
        self.nav_state = 0       # 상태 초기화
        self.arming_state = 0    # 상태 초기화

        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.01)

        # ---------------------------------------------------------
        # 1. Offboard 모드 전환 및 시동 (3단계 로직)
        # ---------------------------------------------------------
        
        # Phase 1: Offboard Heartbeat 전송 (최소 2초 필요)
        print(">>> Phase 1: Sending Offboard heartbeat for 2 seconds...")
        for i in range(20):  # 2초
            self._publish_offboard_control_mode(mode="velocity")
            self._publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.0])
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Phase 2: Offboard 모드 전환
        print(">>> Phase 2: Switching to Offboard mode...")
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        
        # Offboard 모드 확인 (최대 5초 대기)
        for i in range(50):
            self._publish_offboard_control_mode(mode="velocity")
            self._publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.0])
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.nav_state == 14:
                print(f">>> Offboard mode activated! (Attempt: {i})")
                break
        
        if self.nav_state != 14:
            print(">>> Offboard mode switch failed. Retrying...")
            return self.reset(seed=seed)
        
        # Phase 3: Arming
        print(">>> Phase 3: Arming...")
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        
        # Arming 확인 (최대 5초 대기)
        for i in range(50):
            self._publish_offboard_control_mode(mode="velocity")
            self._publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.0])
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.arming_state == 2:
                print(f">>> Armed successfully! (Attempt: {i})")
                break
        
        # 최종 확인
        if self.nav_state != 14 or self.arming_state != 2:
            print(f">>> Arming Failed - Nav: {self.nav_state}, Arm: {self.arming_state}. Retrying...")
            return self.reset(seed=seed)

        print(">>> Returning to Home (Position Control)...")
        takeoff_start = time.time()

        home_target = self.home_pos.tolist()
        
        while True:
            # 1. Position 모드 명령 전송 (yaw=0.0 강제)
            self._publish_offboard_control_mode(mode="position")
            self._publish_trajectory_setpoint(position=home_target, yaw=0.0)
            
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            # 2. 도착 확인 (오차 0.5m 이내)
            dist_to_home = np.linalg.norm(self.current_pos - np.array(home_target))
            
            if dist_to_home < 0.5:
                print(">>> Arrived at Home! Starting Lazy Survival Mode...")
                
                # [2] 2초 안정화 대기
                print(">>> Stabilizing for 2 seconds...")
                stabilize_start = time.time()
                while time.time() - stabilize_start < 2.0:
                    self._publish_offboard_control_mode(mode="position")
                    self._publish_trajectory_setpoint(position=home_target, yaw=0.0)
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                
                # [3] 게임 시작 직전에 사격 개시!
                print(">>> 🔫 Turret ARMED! Combat Start!")
                self._set_turret_status(True)
                
                break

            # 3. 타임아웃 (30초)
            if time.time() - takeoff_start > 30.0:
                print(">>> 🚨 Critical Failure: RTH Timeout! Drone is likely flipped.")
                print(">>> Initiating PX4/Gazebo full restart...")
                
                # PX4/Gazebo 완전 재시작
                reset_success = self._reset_gazebo_world()
                
                if not reset_success:
                    self.reset_retry_count += 1
                    if self.reset_retry_count >= 3:
                        raise RuntimeError("⛔ PX4/Gazebo restart failed after 3 attempts!")
                    print(f">>> Retrying... (Attempt {self.reset_retry_count}/3)")
                    return self.reset(seed=seed)
                
                # 재시작 성공 → 카운터 초기화하고 새로운 에피소드 시작
                self.reset_retry_count = 0
                print(">>> ✅ Restart successful! Starting fresh episode...")
                return self.reset(seed=seed)

        print(">>> Ready to Survive! Lazy Dodge Mode ON.\n")
        
        # 리셋 성공 시 카운터 초기화
        self.reset_retry_count = 0
        
        return self._get_obs(), {}

    def step(self, action):
        start_time = self.node.get_clock().now().nanoseconds
        self.steps += 1
        
        self._publish_offboard_control_mode(mode="velocity")

        real_action = action * self.action_scale_ratio
        self._publish_trajectory_setpoint(velocity=real_action)
        
        # 명령 전송 직후 즉시 데이터 수신 (짧은 timeout)
        rclpy.spin_once(self.node, timeout_sec=0.001)

        target_duration = 100_000_000  # 0.1s in nanoseconds

        # 정확히 0.1초가 될 때까지 대기하면서 데이터 수신
        while (self.node.get_clock().now().nanoseconds - start_time) < target_duration:
            # 대기하는 동안에도 들어오는 데이터(총알, 오도메트리)는 처리해야 함
            rclpy.spin_once(self.node, timeout_sec=0.001)

        # 2. 총알 처리
        closest_bullet_vec, min_dist, is_hit = self._process_bullets()

        # 홈으로부터의 상대 위치 계산
        home_rel = self.home_pos - self.current_pos
        
        # obs 생성
        obs = np.concatenate([home_rel, self.current_vel, closest_bullet_vec])
        obs = obs.astype(np.float32)

        # 3. 보상 계산 (Lazy Survivor)
        reward = 0.0
        done = False
        truncated = False
        
        dist_to_home = np.linalg.norm(home_rel)

        # [1] 생존 보상 (매 스텝)
        reward += 1.0
        
        # 위치 페널티
        #if dist_to_home > 8.0:
        #    reward -= 0.1 * (dist_to_home - 8.0)
        
        # 에너지 패널티
        # 불필요한 움직임 억제 (가만히 있는 게 이득임을 학습)
        energy_penalty = -0.05 * np.linalg.norm(action)
        reward += energy_penalty

        # 중심 거리 페널티
        # 거리제곱 페널티
        reward -= 0.005 * (dist_to_home**2)

        # 근접 회피 보상
        if not is_hit and min_dist < 2.0:
            reward += 1.0  

        # [4] 종료 조건 및 Death 페널티
        if is_hit:
            #CHANGEABLE
            reward -= 500.0  # 피격 시 큰 페널티
            done = True
            print(f"💥 Hit! Survived {self.steps} steps. Home dist: {dist_to_home:.2f}m")
        
        elif self.current_pos[2] > -0.2:  # 지면 20cm 이내 (추락)
            reward -= 500.0
            done = True
            print(f"💥 Crashed! Survived {self.steps} steps.")
        
        elif dist_to_home > self.safe_radius:  # 홈에서 safe_radius 이상 벗어남 (탈영)
            reward -= 500.0
            done = True
            print(f"🚨 Desertion! Too far from home: {dist_to_home:.2f}m (limit: {self.safe_radius}m)")
        
        elif self.steps >= self.max_steps:
            done = True
            truncated = True
            print(f"⏱️ Time's up! Successfully survived {self.max_steps} steps!")

        return obs, reward, done, truncated, {}

    def _get_obs(self):
        """관측값 생성 (홈 상대 위치 기반)"""
        home_rel = self.home_pos - self.current_pos
        dummy_bullet = np.array([100.0, 100.0, 100.0, 0.0, 0.0, 0.0]) 
        return np.concatenate([home_rel, self.current_vel, dummy_bullet]).astype(np.float32)

    def _process_bullets(self):
        """
        1. CCD로 피격 판정 (모든 총알 대상)
        2. Observation용 '가장 위협적인 총알' 선별 (다가오는 총알만!)
        """

        # 안전장치: 데이터 없음 또는 손상된 데이터 (6의 배수 아님)
        if len(self.raw_bullets) == 0 or len(self.raw_bullets) % 6 != 0:
            return np.array([100.0, 100.0, 100.0, 0.0, 0.0, 0.0]), 999.0, False

        num_bullets = len(self.raw_bullets) // 6
        drone_p = self.current_pos
        
        closest_dist = 999.0
        closest_bullet_vec = np.zeros(6)
        hit_detected = False
        
        threat_candidates = []

        # NumPy array로 변환 (성능 최적화)
        bullets_array = np.array(self.raw_bullets).reshape(-1, 6)

        for bullet_data in bullets_array:
            b_curr = bullet_data[:3]
            b_vel = bullet_data[3:]
            
            # 1. 피격 판정 (CCD) - 지나가든 다가오든 맞으면 끝
            # 안전장치: 정지 총알 처리
            speed = np.linalg.norm(b_vel)
            if speed < 0.1:  # 거의 정지 (버그 방지)
                dist_segment = np.linalg.norm(drone_p - b_curr)
            else:
                b_prev = b_curr - (b_vel * self.dt)
                dist_segment = self._point_line_segment_distance(drone_p, b_prev, b_curr)
            
            if dist_segment < 0.5: # 드론 반경 (Iris 프로펠러 포함)
                hit_detected = True
            
            # 2. 위협 후보 선별 (Observation용)
            # 나에게 다가오는가? (내적 > 0)
            vec_b_to_d = drone_p - b_curr # Bullet -> Drone 벡터
            dot_prod = np.dot(b_vel, vec_b_to_d)
            
            if dot_prod > 0: # 다가오는 중 (Approaching)
                dist_curr = np.linalg.norm(vec_b_to_d)
                threat_candidates.append((dist_curr, b_curr, b_vel))

        # 가장 가까운 '다가오는' 위협 선택
        if threat_candidates:
            threat_candidates.sort(key=lambda x: x[0])
            best_threat = threat_candidates[0]
            
            rel_pos = best_threat[1] - drone_p
            closest_bullet_vec = np.concatenate([rel_pos, best_threat[2]])
            closest_dist = best_threat[0]
        else:
            # 위협 없음 (모두 지나갔거나 없음)
            closest_bullet_vec = np.array([100.0, 100.0, 100.0, 0.0, 0.0, 0.0])
            closest_dist = 999.0

        return closest_bullet_vec, closest_dist, hit_detected

    def _point_line_segment_distance(self, point, start, end):
        """점과 선분 사이의 최단 거리 계산 (CCD용)"""
        line_vec = end - start
        point_vec = point - start
        line_len_sq = np.dot(line_vec, line_vec)
        
        if line_len_sq == 0:
            return np.linalg.norm(point_vec)
        
        t = np.dot(point_vec, line_vec) / line_len_sq
        t = max(0.0, min(1.0, t))
        nearest = start + t * line_vec
        return np.linalg.norm(point - nearest)

    def _publish_offboard_control_mode(self, mode="velocity"):
        """Offboard Control Mode 메시지 발행"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        
        if mode == "position":
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
        else: # velocity mode
            msg.position = False
            msg.velocity = True
            msg.acceleration = False
            
        self.pub_offboard_mode.publish(msg)

    def _publish_trajectory_setpoint(self, position=None, velocity=None, yaw=float('nan')):
        """Trajectory Setpoint 메시지 발행"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.yaw = float(yaw)
        
        # 위치 명령이 있으면 위치 채우고, 속도는 NaN
        if position is not None:
            msg.position = [float(position[0]), float(position[1]), float(position[2])]
            msg.velocity = [float('nan')] * 3
        # 속도 명령이 있으면 속도 채우고, 위치는 NaN
        elif velocity is not None:
            msg.position = [float('nan')] * 3
            msg.velocity = [float(velocity[0]), float(velocity[1]), float(velocity[2])]
        else:
            # 안전장치
            msg.position = [float('nan')] * 3
            msg.velocity = [float('nan')] * 3
            
        self.pub_trajectory.publish(msg)

    def _publish_vehicle_command(self, command, p1=0.0, p2=0.0):
        """Vehicle Command 메시지 발행"""
        msg = VehicleCommand()
        msg.command = command; msg.param1 = p1; msg.param2 = p2
        msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1
        msg.from_external = True; msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_command.publish(msg)

    def _reset_gazebo_world(self):
        """
        PX4와 Gazebo를 완전히 재시작하여 시뮬레이션 환경을 초기화합니다.
        이 방법은 뒤집힌 드론이나 복구 불가능한 상태에서도 작동합니다.
        
        Returns:
            bool: 재시작 성공 여부
        """
        import subprocess
        import os
        
        print("\n>>> 🔄 PX4/Gazebo 완전 재시작 시작...")
        
        # 1. 기존 프로세스 모두 종료
        if not self._kill_px4_gazebo():
            print(">>> ⚠️ 프로세스 종료 실패, 계속 진행...")
        
        # 2. MicroXRCE Agent 재시작
        if not self._start_micro_xrce_agent():
            print(">>> ⚠️ Agent 재시작 실패, 계속 진행...")
        
        # 3. PX4/Gazebo 재시작
        if not self._start_px4_gazebo():
            print(">>> ❌ PX4/Gazebo 재시작 실패!")
            return False
        
        # 4. ROS2 연결 대기
        if not self._wait_for_ros2_connection(timeout=30.0):
            print(">>> ❌ ROS2 연결 실패!")
            return False
        
        print(">>> ✅ PX4/Gazebo 재시작 완료! 드론이 원점에 있습니다.")
        return True
    
    def _kill_px4_gazebo(self):
        """PX4와 Gazebo 프로세스 모두 종료"""
        import subprocess
        
        print(">>> 🔴 Killing PX4 and Gazebo processes...")
        
        processes_to_kill = ['px4', 'gzserver', 'gzclient', 'gazebo', 'MicroXRCEAgent']
        
        for proc_name in processes_to_kill:
            try:
                subprocess.run(f"pkill -9 {proc_name}", shell=True, capture_output=True, timeout=2.0)
            except Exception:
                pass
        
        print(">>> Waiting 3 seconds for processes to terminate...")
        time.sleep(3.0)
        print(">>> ✅ All processes killed!")
        return True
    
    def _start_micro_xrce_agent(self):
        """Micro-XRCE-DDS-Agent 시작"""
        import subprocess
        
        print(">>> 🌉 Starting Micro-XRCE-DDS-Agent...")
        
        try:
            self.agent_process = subprocess.Popen(
                ["MicroXRCEAgent", "udp4", "-p", "8888"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            print(f">>> Agent started (PID: {self.agent_process.pid})")
            time.sleep(3.0)
            
            if self.agent_process.poll() is None:
                print(">>> ✅ Agent successfully started!")
                return True
            else:
                return False
                
        except Exception as e:
            print(f">>> ⚠️ Agent start failed: {e}")
            return True  # Agent 실패해도 계속 진행
    
    def _start_px4_gazebo(self):
        """PX4 SITL과 Gazebo 재시작"""
        import subprocess
        import os
        
        print(">>> 🚀 Starting PX4 SITL + Gazebo...")
        
        px4_path = os.path.expanduser("~/PX4-Autopilot")
        
        if not os.path.exists(px4_path):
            print(f">>> ❌ PX4-Autopilot not found at: {px4_path}")
            return False
        
        try:
            cmd = f"cd {px4_path} && HEADLESS=1 PX4_SIM_SPEED_FACTOR=1 make px4_sitl gazebo-classic_iris"
            
            self.px4_process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            print(f">>> PX4 started (PID: {self.px4_process.pid})")
            print(">>> Waiting 15 seconds for initialization...")
            time.sleep(15.0)
            
            if self.px4_process.poll() is None:
                print(">>> ✅ PX4/Gazebo successfully started!")
                return True
            else:
                print(">>> ❌ PX4/Gazebo failed to start!")
                return False
                
        except Exception as e:
            print(f">>> ❌ Failed to start PX4: {e}")
            return False
    
    def _wait_for_ros2_connection(self, timeout=30.0):
        """ROS2 토픽 연결 대기"""
        print(">>> ⏳ Waiting for ROS2 connection...")
        
        start_time = time.time()
        
        # 먼저 상태 변수 초기화
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.nav_state = 0
        self.arming_state = 0
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            
            # 유효한 데이터 수신 확인 (원점 근처에 있어야 함)
            if np.any(np.abs(self.current_pos) > 0.001):
                print(">>> ✅ ROS2 connection established!")
                print(f">>> 현재 위치: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f})")
                return True
            
            # 진행 상황 출력
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed - int(elapsed) < 0.5:
                print(f">>> Waiting... ({int(elapsed)}s / {int(timeout)}s)")
        
        print(">>> ❌ ROS2 connection timeout!")
        return False

    def _wait_for_stable_state(self, timeout=3.0, min_ok=10):
        """
        [Legacy Function - Currently Not Used]
        
        드론의 관측값이 안정될 때까지 대기합니다.
        PX4/Gazebo 재시작 방식으로 변경되면서 사용되지 않지만,
        향후 필요할 수 있어 유지합니다.
        
        Args:
            timeout: 최대 대기 시간 (초)
            min_ok: 연속으로 유효한 관측이 몇 번 나와야 안정으로 판단할지
        
        Raises:
            RuntimeError: 타임아웃 내에 안정화되지 않으면 발생
        """
        t0 = time.time()
        ok_count = 0
        
        while time.time() - t0 < timeout:
            # 현재 관측값 가져오기
            obs = self._get_obs()
            
            # 유효성 검사 (NaN, Inf 체크)
            if np.all(np.isfinite(obs)):
                ok_count += 1
                if ok_count >= min_ok:
                    # 연속으로 min_ok번 유효 → 안정화 완료
                    return
            else:
                # 유효하지 않으면 카운터 리셋
                ok_count = 0
            
            # 짧은 대기
            time.sleep(0.05)  # 50ms
        
        # 타임아웃 발생
        raise RuntimeError(f"State did not stabilize within {timeout}s (ok_count={ok_count}/{min_ok})")
