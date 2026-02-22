#TODO : drone_env.py 에 turret_sim_new.py 적용
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Bool  # Bool 추가
import numpy as np
import time
import random  # [추가] 랜덤 모듈

class TurretSim(Node):
    def __init__(self):
        super().__init__('turret_sim_node')

        # --- [1. 터렛 설정] ---
        # 난이도 1 : 100,50
        # 난이도 2 : 300,150
        # 난이도 3 : 700,350
        self.turret_pos = np.array([700.0, 0.0, 0.0])
        
        self.bullet_speed = 350.0  # 총알 속도 (m/s)
        self.bullet_range = 750.0  # 사거리 (m) - 터렛 거리보다 약간 크게
        
        # [수정 2] 랜덤 발사 간격 설정
        self.fire_rate_min = 1.0  # 최소 1초
        self.fire_rate_max = 3.0  # 최대 3초
        self.next_fire_interval = 2.0  # 첫 발사 간격 초기화
        
        self.dt = 0.01  # 물리 업데이트 주기 (100Hz)

        # --- [2. 상태 변수] ---
        self.bullets = [] # 총알 리스트 [{'pos': vec3, 'vel': vec3}, ...]
        self.drone_pos = np.zeros(3)
        self.drone_vel = np.zeros(3)
        self.last_fire_time = 0.0
        
        # [추가] 사격 허가 플래그 (기본값: False - 안전 제일!)
        self.is_armed = False

        self.arm_time = 0.0          # 사격 허가 받은 시각
        self.first_shot_delay = 0.0  # 첫 발 쏠 때까지 뜸들이는 시간

        # --- [3. 통신 설정] ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        
        # [추가] 사격 허가 명령 수신
        self.enable_sub = self.create_subscription(
            Bool, '/turret/enable', self.enable_callback, 10)

        # [중요] AI에게 보낼 총알 데이터
        # 데이터 형식: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, ...] (6개씩 반복)
        # 속도(vel)를 같이 보내야 드론이 "선분 교차 판정"을 할 수 있음
        self.bullets_pub = self.create_publisher(Float32MultiArray, '/turret/bullets', 10)
        
        # Rviz 시각화용
        self.marker_pub = self.create_publisher(Marker, '/turret/visual', 10)

        # 물리 엔진 타이머 (0.1초마다 실행)
        self.timer = self.create_timer(self.dt, self.update_callback)

    def odom_callback(self, msg):
        self.drone_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.drone_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
    
    def enable_callback(self, msg):
        """사격 허가 스위치 콜백"""
        was_armed = self.is_armed
        self.is_armed = msg.data
        
        # Arming 시 첫 발사 간격 초기화
        if not was_armed and self.is_armed:
            self.next_fire_interval = random.uniform(self.fire_rate_min, self.fire_rate_max)
            self.last_fire_time = time.time()  # 타이머 리셋
        
        # 상태가 바뀌면 로그 출력
        if was_armed != self.is_armed:
            status = "🔫 ARMED" if self.is_armed else "🛑 DISARMED"
            self.get_logger().info(f"Turret Status: {status}")

    def update_callback(self):
        current_time = time.time()

        # -------------------------------------------------
        # 1. 발사 로직 (Fire Logic)
        # [수정 3] 랜덤 간격 적용
        # 고정된 self.fire_rate 대신 매번 바뀌는 self.next_fire_interval 사용
        # -------------------------------------------------
        if self.is_armed and (current_time - self.last_fire_time > self.next_fire_interval):
            self.fire_bullet()
            self.last_fire_time = current_time
            
            # 다음 발사는 언제 쏠지 랜덤으로 결정 (주사위 굴리기 🎲)
            self.next_fire_interval = random.uniform(self.fire_rate_min, self.fire_rate_max)
            # self.get_logger().info(f"Next shot in {self.next_fire_interval:.2f} sec")

        # -------------------------------------------------
        # 2. 총알 물리 업데이트 (Physics)
        # -------------------------------------------------
        active_bullets = []
        flat_list = [] 

        for b in self.bullets:
            # 위치 업데이트: P_new = P_old + V * dt
            b['pos'] += b['vel'] * self.dt
            
            # 유효성 검사 (땅에 박히거나 사거리 벗어나면 제거)
            # NED 좌표계에서: z < 0 (공중), z = 0 (지면), z > 0 (지하)
            if b['pos'][2] <= 0.0:  # 공중에 있는 총알만 체크
                dist = np.linalg.norm(b['pos'] - self.turret_pos)
                
                if dist < self.bullet_range:  # 사거리 내에 있으면 유지
                    active_bullets.append(b)
                    
                    # [데이터 패킹]
                    # 위치(3개) + 속도(3개) = 총 6개 데이터
                    flat_list.extend([
                        float(b['pos'][0]), float(b['pos'][1]), float(b['pos'][2]), # Position
                        float(b['vel'][0]), float(b['vel'][1]), float(b['vel'][2])  # Velocity (for Line Segment check)
                    ])
        
        self.bullets = active_bullets

        # -------------------------------------------------
        # 3. 데이터 전송 (Broadcast)
        # -------------------------------------------------
        msg = Float32MultiArray()
        msg.data = flat_list
        self.bullets_pub.publish(msg)

        # 시각화 업데이트
        self.publish_markers()

    def fire_bullet(self):
        # [예측 사격 알고리즘]
        # 1. 거리 계산
        dist_to_drone = np.linalg.norm(self.drone_pos - self.turret_pos)
        if dist_to_drone < 0.1: dist_to_drone = 0.1
        
        # 2. 도달 시간 계산 (t = 거리 / 속력)
        time_to_hit = dist_to_drone / self.bullet_speed
        
        # 3. 미래 위치 예측 (P_future = P_now + V * t)
        predicted_pos = self.drone_pos + (self.drone_vel * time_to_hit)
        
        # 4. 발사 벡터 계산 (Aim Vector)
        aim_vec = predicted_pos - self.turret_pos
        aim_vec = aim_vec / np.linalg.norm(aim_vec) # 단위 벡터화

        # 5. 총알 생성
        new_bullet = {
            'pos': self.turret_pos.copy(),        # 시작점
            'vel': aim_vec * self.bullet_speed    # 속도 벡터
        }
        self.bullets.append(new_bullet)
        self.get_logger().info(f"Fire! Active Bullets: {len(self.bullets)}")

    def publish_markers(self):
        # 1. 터렛 본체 (Sphere)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "turret_base"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 1.0; marker.scale.y = 1.0; marker.scale.z = 1.0
        marker.color.a = 1.0; marker.color.r = 0.2; marker.color.g = 0.2; marker.color.b = 0.2
        
        # Rviz 좌표계 변환 (NED -> ENU)
        marker.pose.position.x = self.turret_pos[1]
        marker.pose.position.y = self.turret_pos[0]
        marker.pose.position.z = -self.turret_pos[2]
        self.marker_pub.publish(marker)

        # 2. 총알들 (Points)
        b_marker = Marker()
        b_marker.header.frame_id = "map"
        b_marker.ns = "bullets"
        b_marker.id = 1
        b_marker.type = Marker.POINTS
        b_marker.action = Marker.ADD
        b_marker.scale.x = 0.1; b_marker.scale.y = 0.1 # 총알 크기
        b_marker.color.a = 1.0; b_marker.color.r = 1.0; b_marker.color.g = 0.0; b_marker.color.b = 0.0 # 빨간색

        for b in self.bullets:
            p = Point()
            # Rviz 좌표계 변환 (x<->y, z뒤집기)
            p.x = float(b['pos'][1]) 
            p.y = float(b['pos'][0]) 
            p.z = -float(b['pos'][2]) 
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