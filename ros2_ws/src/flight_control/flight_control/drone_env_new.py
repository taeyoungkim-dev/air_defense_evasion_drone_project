import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from std_msgs.msg import Float32MultiArray

class DroneEnv(gym.Env):
    def __init__(self):
        super(DroneEnv, self).__init__()
        
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

        self.sub_odom = self.node.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)
        self.sub_status = self.node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos_profile)
        
        # 터렛 총알 데이터 수신
        self.sub_bullets = self.node.create_subscription(Float32MultiArray, '/turret/bullets', self.bullets_cb, 10)

        self.nav_state = 0
        self.arming_state = 0

        # 4. 변수 초기화
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        #TODO
        self.target_pos = np.array([10.0, 0.0, -2.0]) # 목표 지점 (2m 높이)
        self.raw_bullets = ()  # tuple로 초기화 (타입 일관성)
        
        # 리셋 재시도 카운터 (무한 루프 방지)
        self.reset_retry_count = 0 

        # 최대 스텝 (30초)
        self.max_steps = 300
        self.steps = 0
        self.dt = 0.1

        # 5. Gym Space 정의
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        # Obs: [Target(3), Self_Vel(3), Bullet_Rel(3), Bullet_Vel(3)]
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)

        self.prev_pos = np.zeros(3) # 초기화

    # --- Callbacks ---
    def odom_cb(self, msg):
        self.current_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.current_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])

    def status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def bullets_cb(self, msg):
        try:
            # "야, 뭐가 들어오든 일단 튜플로 바꿔봐. 안 되면 에러 내지 말고 무시해."
            self.raw_bullets = tuple(msg.data)
        except Exception:
            # "데이터가 꼬였네? 이번 한 번은 그냥 빈 걸로 치자."
            self.raw_bullets = ()

    # --- Gym Methods ---
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        print("\n>>> Resetting Environment...")
        
        # 무한 재시도 방지
        if self.reset_retry_count >= 3:
            self.reset_retry_count = 0  # 리셋
            raise RuntimeError("⛔ Reset failed after 3 attempts! Manual intervention required.")
        
        self.steps = 0
        self.current_pos = np.zeros(3)
        self.prev_pos = np.zeros(3)
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

        takeoff_start = time.time()
        
        print(">>> Returning to Home (Position Control)...")
        takeoff_start = time.time()

        home_target = [0.0, 0.0, -2.0]
        
        while True:
            # 1. Position 모드 명령 전송
            self._publish_offboard_control_mode(mode="position")
            self._publish_trajectory_setpoint(position=home_target,yaw=0.0)
            
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            # 2. 도착 확인 (오차 0.5m 이내)
            dist_to_home = np.linalg.norm(self.current_pos - np.array(home_target))
            
            if dist_to_home < 0.5:
                print(">>> Arrived at Home! Starting Episode...")
                break

            # 3. 타임아웃 (30초)
            if time.time() - takeoff_start > 30.0:
                print(">>> 🚨 Critical Failure: RTH Timeout! Drone is likely flipped.")
                
                # [조치 1] 강제 시동 끄기 (Force Disarm)
                print(">>> Action 1: Force Disarm...")
                self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
                
                # Disarm 완료 확인 (최대 2초 대기)
                for _ in range(20):
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                    if self.arming_state != 2:  # Armed 아님
                        print(">>> Disarm confirmed!")
                        break

                # [조치 2] 시뮬레이터 강제 리셋 (텔레포트)
                print(">>> Action 2: Teleporting to Home via Gazebo...")
                teleport_success = self._try_force_reset_pose()
                
                if not teleport_success:
                    print(">>> ⚠️ Teleport failed, but continuing reset...")

                # [조치 3] 재시도
                self.reset_retry_count += 1
                print(f">>> Action 3: Restarting Episode (Attempt {self.reset_retry_count}/3)...")
                return self.reset(seed=seed)

        print(">>> Ready to Fly! Efficient Dodge Mode ON.\n")
        
        # 리셋 성공 시 카운터 초기화
        self.reset_retry_count = 0
        
        return self._get_obs(), {}

    def step(self, action):
        self.steps += 1
        
        self._publish_offboard_control_mode(mode="velocity")
        real_action = action * 10.0 
        self._publish_trajectory_setpoint(velocity=real_action)
        rclpy.spin_once(self.node, timeout_sec=self.dt)

        # 2. 총알 처리
        closest_bullet_vec, min_dist, is_hit = self._process_bullets()

        target_rel = self.target_pos - self.current_pos
        # obs에 dummy_bullet 대신 실제 closest_bullet_vec 사용
        obs = np.concatenate([target_rel, self.current_vel, closest_bullet_vec])
        obs = obs.astype(np.float32)

        # 3. 보상 계산
        reward = 0.0
        done = False
        truncated = False
        
        dist_to_target = np.linalg.norm(target_rel) # 미리 계산

        # [1] 전진 보상 (Progress Reward)
        # prev_pos가 초기화되어 있어야 함
        prev_dist = np.linalg.norm(self.prev_pos - self.target_pos)
        curr_dist = dist_to_target
        
        progress = prev_dist - curr_dist
        if progress > 0:
            reward += progress * 30.0 
        
        # [2] 거리 벌점 (약하게)
        reward -= curr_dist * 0.05
        
        # [3] 회피 보상 (Near Miss)
        if not is_hit and 0.5 < min_dist < 1.0:
            reward += 1.0 

        # [4] 이벤트 처리
        if is_hit:
            reward -= 50.0 # 패널티 완화
            done = True
            print(f"Hit! (Dist: {curr_dist:.1f}m)")
        
        elif dist_to_target < 2.0:
            reward += 1000.0
            done = True
            print("Target Reached!")
        
        elif self.current_pos[2] > -0.2:
             reward -= 50.0 # 추락 패널티 완화
             done = True
             print("Crashed on Ground!")

        # [5] 탈영 방지 (50m 밖으로 나가면 처형)
        elif curr_dist > 50.0:
            reward -= 200.0 
            done = True
            print(f"Desertion! (Dist: {curr_dist:.1f}m)")
        
        elif self.steps >= self.max_steps:
            done = True
            truncated = True

        # [중요] 다음 스텝을 위해 위치 저장
        self.prev_pos = self.current_pos.copy()

        return obs, reward, done, truncated, {}

    def _get_obs(self):
        target_rel = self.target_pos - self.current_pos
        dummy_bullet = np.array([100.0, 100.0, 100.0, 0.0, 0.0, 0.0]) 
        return np.concatenate([target_rel, self.current_vel, dummy_bullet]).astype(np.float32)

    def _process_bullets(self):
        """
        1. CCD로 피격 판정 (모든 총알 대상)
        2. Observation용 '가장 위협적인 총알' 선별 (다가오는 총알만!)
        """
        if self.steps < 30:
            return np.array([100.0]*3 + [0.0]*3), 999.0, False
        
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
        msg = VehicleCommand()
        msg.command = command; msg.param1 = p1; msg.param2 = p2
        msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1
        msg.from_external = True; msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_command.publish(msg)

    def _try_force_reset_pose(self):
        """
        물리적으로 뒤집힌 드론을 Gazebo 명령어로 강제 복구시킵니다.
        
        Returns:
            bool: 텔레포트 성공 여부
        """
        import subprocess
        
        # Gazebo Classic 사용 시: /gazebo/set_model_state 서비스 호출
        # 주의: 모델 이름이 'iris'가 아니라면 수정 필요 (보통 'iris' 또는 'plane')
        try:
            # Gazebo Classic의 올바른 서비스 형식
            cmd = """ros2 service call /gazebo/set_model_state gazebo_msgs/srv/SetModelState "{model_state: {model_name: 'iris', pose: {position: {x: 0.0, y: 0.0, z: 0.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" """
            
            # 동기 호출로 변경 (성공 여부 확인)
            result = subprocess.run(
                cmd, 
                shell=True, 
                capture_output=True, 
                timeout=3.0, 
                text=True
            )
            
            if result.returncode == 0:
                print(">>> ✅ Teleport successful!")
                time.sleep(1.0)  # 안정화 대기
                return True
            else:
                print(f">>> ⚠️ Teleport failed (code {result.returncode}): {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            print(">>> ⚠️ Teleport timeout (service not responding)")
            return False
        except Exception as e:
            print(f">>> ⚠️ Teleport exception: {e}")
            return False
            print(">>> If you use Gazebo Ignition, the command is different.")