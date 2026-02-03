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

    # --- Gym Methods ---
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        print(">>> Resetting & Auto-Takeoff...")
        
        # 0. 변수 초기화
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        
        # 1. 아까 만든 Arming 로직 (시동 걸기)
        armed = False
        for i in range(50):
            self._publish_velocity([0.0, 0.0, 0.0])
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.nav_state == 14 and self.arming_state == 2:
                armed = True
                break
        
        if not armed:
            print(">>> Arming Failed. Retrying...")
            return self.reset(seed=seed) # 재귀 호출로 다시 시도

        # ---------------------------------------------------------
        # [추가된 핵심] 2. 강제 이륙 (공중 2m까지 올리기)
        # ---------------------------------------------------------
        print(">>> Taking off to 2.0m...")
        takeoff_start = time.time()
        
        # 높이가 1.8m(-1.8)에 도달할 때까지 강제로 상승 명령
        while self.current_pos[2] > -1.8: 
            # 상승 속도 -1.0 m/s
            self._publish_velocity([0.0, 0.0, -1.0]) 
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            # 혹시 5초 넘게 못 뜨면 리셋 (무한루프 방지)
            if time.time() - takeoff_start > 5.0:
                print(">>> Takeoff Timeout!")
                return self.reset(seed=seed)

        print(">>> Ready to Fly! Handing over control to AI.")
        return self._get_obs(), {}

    def step(self, action):
        # 1. Action 수행 (속도 명령 전송)
        # Action값(-1~1)을 실제 속도(m/s)로 변환 (예: 최대 10m/s)
        real_action = action * 10.0 
        self._publish_velocity(real_action)

        # 2. ROS2 통신 업데이트 (중요: 데이터를 받을 때까지 잠깐 기다림)
        # spin_once를 여러 번 호출하여 콜백을 처리
        rclpy.spin_once(self.node, timeout_sec=self.dt)

        # 3. Observation 가져오기
        obs = self._get_obs()

        # 4. 보상 계산 (Reward Function)
        reward = 0.0
        done = False
        truncated = False
        
        # (1) 목표와의 거리
        dist_to_target = np.linalg.norm(self.target_pos - self.current_pos)
        
        # (2) 보상: 목표에 가까울수록 좋음 (거리 페널티)
        reward -= dist_to_target * 0.01 
        
        # (3) 종료 조건
        if dist_to_target < 2.0: # 목표 도달
            reward += 100.0
            done = True
            print("Target Reached!")
        
        # 땅에 박으면 종료 (z는 위쪽이 음수, 땅은 0 근처)
        if self.current_pos[2] > 0.5: # 땅보다 아래(실제론 양수)면 추락으로 간주
             reward -= 100.0
             done = True
             print("Crashed!")

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

    def _publish_velocity(self, vel):
        # Heartbeat (필수)
        self._publish_offboard_control_mode()
        
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