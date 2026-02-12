import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

class TurretSim(Node):
    def __init__(self):
        super().__init__('turret_sim_node')

        # --- [1. 터렛 설정] ---
        # 위치: (x=20, y=5) - 출발지(0,0)과 목표지(50,0) 사이에서 약간 비껴난 곳
        self.turret_pos = np.array([20.0, 5.0, 0.0]) 
        
        self.bullet_speed = 50.0  # 총알 속도 (m/s) - 초기 학습용
        self.fire_rate = 0.5      # 발사 주기 (초) - 0.5초당 1발 (120 RPM)
        self.bullet_range = 100.0 # 사거리 (m)
        self.dt = 0.1             # 물리 업데이트 주기 (10Hz)

        # --- [2. 상태 변수] ---
        self.bullets = [] # 총알 리스트 [{'pos': vec3, 'vel': vec3}, ...]
        self.drone_pos = np.zeros(3)
        self.drone_vel = np.zeros(3)
        self.last_fire_time = 0.0

        # --- [3. 통신 설정] ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

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

    def update_callback(self):
        current_time = time.time()

        # -------------------------------------------------
        # 1. 발사 로직 (Fire Logic)
        # -------------------------------------------------
        if current_time - self.last_fire_time > self.fire_rate:
            self.fire_bullet()
            self.last_fire_time = current_time

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
            if b['pos'][2] < 0.0:  # 공중에 있는 총알만 체크
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
        b_marker.scale.x = 0.3; b_marker.scale.y = 0.3 # 총알 크기
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