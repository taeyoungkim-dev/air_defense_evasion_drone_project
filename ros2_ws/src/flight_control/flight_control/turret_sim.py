import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray  # <--- 추가된 부분: 데이터 전송용
import numpy as np

class TurretSim(Node):
    def __init__(self):
        super().__init__('turret_sim_node')

        # 1. 터렛 설정
        self.turret_pos = np.array([20.0, 5.0, 0.0]) # 터렛의 절대 위치
        self.bullet_speed = 30.0 

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        # 시각화용 (Rviz)
        self.marker_pub = self.create_publisher(Marker, '/turret/aim_line', 10)
        
        # AI 입력용 (Data) - [터렛상대위치(3), 조준벡터(3)]
        self.threat_pub = self.create_publisher(Float32MultiArray, '/turret/threat_info', 10)

    def odom_callback(self, msg):
        # 1. 드론 상태 추출 (PX4 NED 좌표계)
        drone_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        drone_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])

        # 2. Lead Shot 계산
        dist = np.linalg.norm(drone_pos - self.turret_pos)
        if dist < 1e-6: dist = 1.0 # 0 나누기 방지
        
        time_to_hit = dist / self.bullet_speed
        predicted_pos = drone_pos + (drone_vel * time_to_hit)

        # 3. 조준 벡터 (Aim Vector) 계산 (단위 벡터)
        aim_vector = predicted_pos - self.turret_pos
        aim_vector = aim_vector / np.linalg.norm(aim_vector) # 정규화

        # ---------------------------------------------------------
        # [핵심] AI에게 보낼 데이터 가공 (Simulating Vision Sensor)
        # ---------------------------------------------------------
        # 드론 입장에서 본 터렛의 상대 위치 (Turret - Drone)
        rel_pos = self.turret_pos - drone_pos

        threat_msg = Float32MultiArray()
        # 데이터 순서: [상대위치X, Y, Z, 조준벡터X, Y, Z] (총 6개)
        threat_msg.data = [
            float(rel_pos[0]), float(rel_pos[1]), float(rel_pos[2]),
            float(aim_vector[0]), float(aim_vector[1]), float(aim_vector[2])
        ]
        self.threat_pub.publish(threat_msg)

        # ---------------------------------------------------------
        # Rviz 시각화 (기존 코드 유지)
        # ---------------------------------------------------------
        self.publish_marker(self.turret_pos, predicted_pos)

    def publish_marker(self, start, end):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0

        p1 = Point(); p1.x, p1.y, p1.z = float(start[1]), float(start[0]), -float(start[2])
        p2 = Point(); p2.x, p2.y, p2.z = float(end[1]), float(end[0]), -float(end[2])
        
        marker.points.append(p1); marker.points.append(p2)
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TurretSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()