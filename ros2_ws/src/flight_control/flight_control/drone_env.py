import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from std_msgs.msg import Float32MultiArray

class DroneEnv(gym.Env):
    def __init__(self):
        super(DroneEnv, self).__init__()
        # 300스탭 안에 목표에 도달하지 못하면 죽음
        # 목표 도달 시 100점, 죽으면 -100점
        # 드론을 재촉하는 용도
        self.max_steps = 300
        self.steps = 0
        # 1. ROS2 노드 초기화 (Env가 곧 Node 역할을 겸함)
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
        # [Action] 드론에게 명령
        self.pub_offboard_mode = self.node.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.pub_trajectory = self.node.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.pub_vehicle_command = self.node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # [Observation] 데이터 수신
        self.sub_odom = self.node.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)
        self.sub_threat = self.node.create_subscription(Float32MultiArray, '/turret/threat_info', self.threat_cb, 10)
        self.sub_status = self.node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos_profile)
        self.nav_state = 0
        self.arming_state = 0

        # 4. 변수 초기화
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.threat_data = np.zeros(6) # [rel_pos(3), aim_vec(3)]
        self.target_pos = np.array([50.0, 0.0, -5.0]) # 목표 지점 (임시: 50m 앞)

        # 5. Gym Space 정의
        # Action: [vx, vy, vz] (각각 -1.0 ~ 1.0 범위로 정규화)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        
        # Observation: [Target_Rel(3), Self_Vel(3), Threat(6)] = 총 12개
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)

        # 6. 제어 주기 (10Hz)
        self.dt = 0.1 

    # --- Callbacks ---
    def odom_cb(self, msg):
        self.current_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.current_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])

    def threat_cb(self, msg):
        self.threat_data = np.array(msg.data)
    
    def status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    # --- Gym Methods ---
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        print("\n>>> Resetting Environment...")
        
        # 0. 변수 초기화
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.nav_state = 0
        self.arming_state = 0

        # 중요: 상태 정보가 들어올 때까지 충분히 대기
        print(">>> Waiting for initial state...")
        time.sleep(0.5)  # 최소 0.5초 대기
        for _ in range(20):  # 2초 동안 상태 수신
            rclpy.spin_once(self.node, timeout_sec=0.1)
        print(f">>> Initial State - Nav: {self.nav_state}, Arm: {self.arming_state}")

        # ---------------------------------------------------------
        # 1. Offboard 모드 전환 및 시동 (3단계 로직)
        # ---------------------------------------------------------
        
        # Phase 1: Offboard Heartbeat 전송 (최소 2초 필요)
        print(">>> Phase 1: Sending Offboard heartbeat for 2 seconds...")
        for i in range(20):  # 2초
            self._publish_offboard_control_mode()
            self._publish_trajectory_setpoint([0.0, 0.0, 0.0])
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Phase 2: Offboard 모드 전환
        print(">>> Phase 2: Switching to Offboard mode...")
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        
        # Offboard 모드 확인 (최대 5초 대기)
        for i in range(50):
            self._publish_offboard_control_mode()
            self._publish_trajectory_setpoint([0.0, 0.0, 0.0])
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
            self._publish_offboard_control_mode()
            self._publish_trajectory_setpoint([0.0, 0.0, 0.0])
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.arming_state == 2:
                print(f">>> Armed successfully! (Attempt: {i})")
                break
        
        # 최종 확인
        if self.nav_state != 14 or self.arming_state != 2:
            print(f">>> Arming Failed - Nav: {self.nav_state}, Arm: {self.arming_state}. Retrying...")
            return self.reset(seed=seed)

        # ---------------------------------------------------------
        # 2. 강제 이륙 (Auto-Takeoff) - 공중 2m까지
        # ---------------------------------------------------------
        print(">>> Auto-Takeoff to 2.0m...")
        takeoff_start = time.time()
        
        while self.current_pos[2] > -1.8: # NED 좌표계: 위쪽이 음수
            # 상승 속도 -1.0 m/s
            self._publish_offboard_control_mode() # Heartbeat
            self._publish_trajectory_setpoint([0.0, 0.0, -1.0]) # Velocity Z = -1.0 (Up)
            
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            # 5초 타임아웃
            if time.time() - takeoff_start > 5.0:
                print(">>> Takeoff Timeout! Retrying...")
                return self.reset(seed=seed)
        self.steps = 0
        print(">>> Ready to Fly! Handing over control to AI.\n")
        return self._get_obs(), {}

    def step(self, action):
        # 1. Heartbeat (필수)
        self._publish_offboard_control_mode()
        
        # 2. Action 수행 (속도 명령 전송)
        real_action = action * 10.0 
        self._publish_trajectory_setpoint(real_action) # 함수 이름 변경됨

        # 3. ROS2 통신 업데이트
        rclpy.spin_once(self.node, timeout_sec=self.dt)

        # 4. Observation 가져오기
        obs = self._get_obs()

        # 5. 보상 계산
        self.steps += 1
        reward = 0.0
        done = False
        truncated = False
        
        dist_to_target = np.linalg.norm(self.target_pos - self.current_pos)
        reward -= dist_to_target * 0.1 # penality ratio
        
        if dist_to_target < 2.0:
            reward += 100.0
            done = True
            print("Target Reached!")
        
        if self.current_pos[2] > 0.5:
             reward -= 100.0
             done = True
             print("Crashed!")
             
        elif self.steps >= self.max_steps:
            done = True
            truncated = True # 시간 초과로 인한 종료임을 명시
            reward -= 50.0   # 시간 초과 페널티 (게으름 벌점)
            # print("Time Out!") # 로그 너무 많이 뜨면 주석 처리

        return obs, reward, done, truncated, {}

    def _get_obs(self):
        # Target Relative Position
        target_rel = self.target_pos - self.current_pos
        
        # [Target(3), Self_Vel(3), Threat(6)]
        obs = np.concatenate([target_rel, self.current_vel, self.threat_data])
        return obs.astype(np.float32)

    # --- Helper Methods ---
    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False; msg.velocity = True; msg.acceleration = False
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.pub_offboard_mode.publish(msg)

    def _publish_trajectory_setpoint(self, vel):
        msg = TrajectorySetpoint()
        msg.position = [float('nan')]*3
        msg.velocity = [float(vel[0]), float(vel[1]), float(vel[2])]
        msg.yaw = float('nan')
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.pub_trajectory.publish(msg)

    def _publish_vehicle_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command; msg.param1 = p1; msg.param2 = p2
        msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_command.publish(msg)