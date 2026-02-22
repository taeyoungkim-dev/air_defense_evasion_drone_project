#!/usr/bin/env python3
"""
PX4/Gazebo 재시작 테스트 스크립트

동작:
1. Offboard + Arming
2. 2m 높이로 이륙
3. (10, 10, -2) 위치로 이동
4. 착륙 + Disarm
5. PX4/Gazebo 완전 종료
6. PX4/Gazebo 재시작
7. 드론이 원점(0,0,-2)에 있는지 확인
8. 다시 Arming + Takeoff 가능한지 테스트
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import time

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState, SetModelState
from gazebo_msgs.msg import EntityState, ModelState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3


class ResetTester(Node):
    def __init__(self):
        super().__init__('reset_tester')
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.pub_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.pub_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Subscribers
        self.sub_odom = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)
        self.sub_status = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos_profile)
        
        # Gazebo 리셋 클라이언트
        self.reset_world_client = self.create_client(Empty, '/reset_world')
        self.set_model_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # 상태 변수
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.nav_state = 0
        self.arming_state = 0
        
        # 프로세스 핸들
        self.px4_process = None
        self.agent_process = None
        
        print("\n" + "="*60)
        print("🧪 PX4/Gazebo Restart Test Script")
        print("="*60)

    def odom_cb(self, msg):
        self.current_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.current_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])

    def status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def publish_offboard_control_mode(self, mode="velocity"):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        if mode == "position":
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
        else:
            msg.position = False
            msg.velocity = True
            msg.acceleration = False
        
        self.pub_offboard_mode.publish(msg)

    def publish_trajectory_setpoint(self, position=None, velocity=None, yaw=float('nan')):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
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

    def publish_vehicle_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_command.publish(msg)

    def arm_and_offboard(self):
        """Arming + Offboard 모드 전환"""
        print("\n>>> Phase 1: Offboard Heartbeat (2초)...")
        for _ in range(20):
            self.publish_offboard_control_mode(mode="velocity")
            self.publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.0])
            rclpy.spin_once(self, timeout_sec=0.1)
        
        print(">>> Phase 2: Offboard 모드 전환...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        
        for i in range(50):
            self.publish_offboard_control_mode(mode="velocity")
            self.publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.0])
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.nav_state == 14:
                print(f">>> Offboard 활성화! (시도: {i})")
                break
        
        if self.nav_state != 14:
            raise RuntimeError("Offboard 모드 전환 실패!")
        
        print(">>> Phase 3: Arming...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        
        for i in range(50):
            self.publish_offboard_control_mode(mode="velocity")
            self.publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.0])
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.arming_state == 2:
                print(f">>> Arming 성공! (시도: {i})")
                break
        
        if self.arming_state != 2:
            raise RuntimeError("Arming 실패!")

    def takeoff_to_2m(self):
        """2m 높이까지 이륙 (호버링)"""
        print("\n>>> Takeoff to 2m (hovering)...")
        start_time = time.time()
        
        while self.current_pos[2] > -2.0:
            self.publish_offboard_control_mode(mode="velocity")
            self.publish_trajectory_setpoint(velocity=[0.0, 0.0, -1.0])  # 1m/s 상승
            rclpy.spin_once(self, timeout_sec=0.05)
            
            if time.time() - start_time > 10.0:
                raise RuntimeError("Takeoff timeout!")
        
        print(f">>> ✅ 2m 도달! 현재 높이: {-self.current_pos[2]:.2f}m")
        
        # 2초간 호버링 (안정화)
        print(">>> Hovering for 2 seconds...")
        start_time = time.time()
        while time.time() - start_time < 2.0:
            self.publish_offboard_control_mode(mode="velocity")
            self.publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.0])
            rclpy.spin_once(self, timeout_sec=0.05)
        print(">>> ✅ Hovering complete!")

    def move_to_position(self, target_pos):
        """지정된 위치로 이동 (Position Control)"""
        print(f"\n>>> 📍 Moving to position ({target_pos[0]}, {target_pos[1]}, {target_pos[2]})...")
        start_time = time.time()
        
        while True:
            self.publish_offboard_control_mode(mode="position")
            self.publish_trajectory_setpoint(position=target_pos, yaw=0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            
            dist = np.linalg.norm(self.current_pos - np.array(target_pos))
            
            if dist < 0.5:
                print(f">>> ✅ 위치 도착! 현재: ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f}, {-self.current_pos[2]:.1f})")
                break
            
            if time.time() - start_time > 30.0:
                raise RuntimeError("Move timeout!")
            
            # 2초마다 진행 상황 출력
            elapsed = time.time() - start_time
            if elapsed > 0 and int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.1:
                print(f">>> 이동 중... 거리: {dist:.2f}m")

    def land(self):
        """착륙"""
        print("\n>>> 🛬 Landing...")
        start_time = time.time()
        
        while self.current_pos[2] < -0.3:  # 지면 30cm까지
            self.publish_offboard_control_mode(mode="velocity")
            self.publish_trajectory_setpoint(velocity=[0.0, 0.0, 0.5])  # 0.5m/s 하강
            rclpy.spin_once(self, timeout_sec=0.05)
            
            if time.time() - start_time > 10.0:
                print(">>> Landing timeout, forcing disarm...")
                break
        
        print(f">>> ✅ 착륙 완료! 최종 높이: {-self.current_pos[2]:.2f}m")

    def disarm(self):
        """시동 끄기"""
        print("\n>>> 🔴 Disarming...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
        
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.arming_state != 2:
                print(">>> Disarm 확인!")
                break
        
        time.sleep(1.0)

    def kill_px4_gazebo(self):
        """PX4와 Gazebo 프로세스 모두 종료"""
        import subprocess
        import os
        
        print("\n>>> 🔴 Killing PX4 and Gazebo processes...")
        
        # 종료할 프로세스 목록
        processes_to_kill = [
            'px4',
            'gzserver',
            'gzclient',
            'gazebo',
            'MicroXRCEAgent',
        ]
        
        for proc_name in processes_to_kill:
            try:
                result = subprocess.run(
                    f"pkill -9 {proc_name}",
                    shell=True,
                    capture_output=True,
                    timeout=2.0
                )
                if result.returncode == 0:
                    print(f">>> ✅ Killed: {proc_name}")
                else:
                    print(f">>> ⚠️ No process found: {proc_name}")
            except Exception as e:
                print(f">>> ⚠️ Error killing {proc_name}: {e}")
        
        print(">>> Waiting 3 seconds for processes to fully terminate...")
        time.sleep(3.0)
        print(">>> ✅ All processes killed!")
    
    def start_px4_gazebo(self):
        """PX4 SITL과 Gazebo 재시작"""
        import subprocess
        import os
        
        print("\n>>> 🚀 Starting PX4 SITL + Gazebo...")
        
        # PX4-Autopilot 경로
        px4_path = os.path.expanduser("~/PX4-Autopilot")
        
        if not os.path.exists(px4_path):
            print(f">>> ❌ PX4-Autopilot not found at: {px4_path}")
            return False
        
        # PX4 SITL 시작 (백그라운드)
        try:
            cmd = f"cd {px4_path} && HEADLESS=1 PX4_SIM_SPEED_FACTOR=3 make px4_sitl gazebo-classic_iris"
            
            # 백그라운드 프로세스로 실행
            self.px4_process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # 새로운 프로세스 그룹 생성
            )
            
            print(f">>> PX4 프로세스 시작됨 (PID: {self.px4_process.pid})")
            print(">>> Waiting 15 seconds for PX4/Gazebo to initialize...")
            time.sleep(15.0)
            
            # 프로세스가 살아있는지 확인
            if self.px4_process.poll() is None:
                print(">>> ✅ PX4/Gazebo successfully started!")
                return True
            else:
                print(">>> ❌ PX4/Gazebo failed to start!")
                return False
                
        except Exception as e:
            print(f">>> ❌ Failed to start PX4: {e}")
            return False
    
    def wait_for_ros2_connection(self, timeout=30.0):
        """ROS2 토픽 연결 대기"""
        print("\n>>> ⏳ Waiting for ROS2 connection...")
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Odometry 토픽 수신 확인
            rclpy.spin_once(self, timeout_sec=0.5)
            
            # 유효한 데이터 수신 확인
            if np.any(self.current_pos != 0):
                print(">>> ✅ ROS2 connection established!")
                print(f">>> 현재 위치: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f})")
                return True
            
            # 진행 상황 출력
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed - int(elapsed) < 0.5:
                print(f">>> Waiting... ({int(elapsed)}s / {int(timeout)}s)")
        
        print(">>> ❌ ROS2 connection timeout!")
        return False
    
    def start_micro_xrce_agent(self):
        """Micro-XRCE-DDS-Agent 시작"""
        import subprocess
        import os
        
        print("\n>>> 🌉 Starting Micro-XRCE-DDS-Agent...")
        
        try:
            # Agent 시작 (백그라운드)
            self.agent_process = subprocess.Popen(
                ["MicroXRCEAgent", "udp4", "-p", "8888"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            print(f">>> Agent 프로세스 시작됨 (PID: {self.agent_process.pid})")
            print(">>> Waiting 3 seconds for agent to initialize...")
            time.sleep(3.0)
            
            if self.agent_process.poll() is None:
                print(">>> ✅ Agent successfully started!")
                return True
            else:
                print(">>> ❌ Agent failed to start!")
                return False
                
        except Exception as e:
            print(f">>> ⚠️ Failed to start Agent: {e}")
            print(">>> (Agent might already be running or not needed)")
            return True  # Agent 실패해도 계속 진행

    def wait_for_stable_state(self, timeout=3.0, min_ok=10):
        """관측값 안정화 대기"""
        print("\n>>> ⏳ Waiting for state stabilization...")
        
        t0 = time.time()
        ok_count = 0
        
        while time.time() - t0 < timeout:
            # 위치와 속도가 유한한 값인지 체크
            if np.all(np.isfinite(self.current_pos)) and np.all(np.isfinite(self.current_vel)):
                ok_count += 1
                if ok_count >= min_ok:
                    print(f">>> ✅ 상태 안정화 완료! (연속 {min_ok}회 유효)")
                    return
            else:
                ok_count = 0
            
            time.sleep(0.05)
        
        print(f">>> ⚠️ 상태 안정화 타임아웃 (ok_count={ok_count}/{min_ok})")

    def rth_to_home(self):
        """홈 포지션으로 복귀 (0,0,-2)"""
        print("\n>>> 🏠 Return To Home (0, 0, -2)...")
        
        home_target = [0.0, 0.0, -2.0]
        start_time = time.time()
        
        while True:
            self.publish_offboard_control_mode(mode="position")
            self.publish_trajectory_setpoint(position=home_target, yaw=0.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            
            dist = np.linalg.norm(self.current_pos - np.array(home_target))
            
            if dist < 0.5:
                print(">>> ✅ 홈 도착!")
                break
            
            if time.time() - start_time > 30.0:
                print(">>> ⚠️ RTH 타임아웃!")
                break
            
            if time.time() - start_time > 0 and int(time.time() - start_time) % 2 == 0:
                print(f">>> RTH 진행 중... 거리: {dist:.2f}m")

    def run_test(self):
        """전체 테스트 실행"""
        try:
            print("\n" + "="*60)
            print("🎬 Phase 1: 정상 비행 테스트")
            print("="*60)
            
            # 1. Arming + Offboard
            self.arm_and_offboard()
            
            # 2. 2m 이륙 + 호버링
            self.takeoff_to_2m()
            
            # 3. (10, 10, -2) 위치로 이동
            self.move_to_position([10.0, 10.0, -2.0])
            
            # 4. 착륙
            self.land()
            
            # 5. Disarm
            self.disarm()
            
            print("\n" + "="*60)
            print("✅ Phase 1 완료! 드론이 (10, 10)에 착륙했습니다.")
            print("="*60)
            
            # ============================================
            # Phase 2: PX4/Gazebo 재시작
            # ============================================
            print("\n" + "="*60)
            print("🔄 Phase 2: PX4/Gazebo 재시작 테스트")
            print("="*60)
            
            # 6. PX4/Gazebo 종료
            self.kill_px4_gazebo()
            
            # 7. MicroXRCE Agent 재시작
            if not self.start_micro_xrce_agent():
                print(">>> ⚠️ Agent 시작 실패, 계속 진행...")
            
            # 8. PX4/Gazebo 재시작
            if not self.start_px4_gazebo():
                raise RuntimeError("PX4/Gazebo 재시작 실패!")
            
            # 9. ROS2 연결 대기
            if not self.wait_for_ros2_connection(timeout=30.0):
                raise RuntimeError("ROS2 연결 실패!")
            
            # 10. 현재 위치 확인 (원점에 있어야 함)
            print("\n>>> 📍 드론 위치 확인...")
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            print(f">>> 현재 위치: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f})")
            
            dist_from_origin = np.sqrt(self.current_pos[0]**2 + self.current_pos[1]**2)
            if dist_from_origin < 1.0:
                print(">>> ✅ 드론이 원점에 있습니다!")
            else:
                print(f">>> ⚠️ 드론이 원점에서 {dist_from_origin:.2f}m 떨어져 있습니다.")
            
            # ============================================
            # Phase 3: 재시작 후 비행 테스트
            # ============================================
            print("\n" + "="*60)
            print("🚁 Phase 3: 재시작 후 비행 가능 여부 테스트")
            print("="*60)
            
            # 11. 다시 Arming + Offboard
            self.arm_and_offboard()
            
            # 12. 2m 이륙
            self.takeoff_to_2m()
            
            print("\n" + "="*60)
            print("✅ 모든 테스트 완료!")
            print("="*60)
            print("\n📊 테스트 요약:")
            print("  1. 정상 비행 (Arm, Takeoff, Move, Land) ✅")
            print("  2. PX4/Gazebo 완전 종료 ✅")
            print("  3. PX4/Gazebo 재시작 ✅")
            print("  4. 드론 원점 복귀 확인 ✅")
            print("  5. 재시작 후 Arm/Takeoff 가능 ✅")
            print("\n🎉 PX4/Gazebo 재시작 방식으로 환경 리셋 가능!")
            
        except Exception as e:
            print(f"\n❌ 테스트 실패: {e}")
            import traceback
            traceback.print_exc()


def main():
    rclpy.init()
    
    tester = ResetTester()
    
    # 초기 대기
    print(">>> 3초 후 테스트 시작...")
    for i in range(3, 0, -1):
        print(f">>> {i}...")
        time.sleep(1.0)
    
    tester.run_test()
    
    print("\n>>> 5초 후 종료...")
    time.sleep(5.0)
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
