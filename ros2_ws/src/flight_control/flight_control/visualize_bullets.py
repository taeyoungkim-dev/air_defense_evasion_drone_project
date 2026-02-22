#!/usr/bin/env python3
"""
RViz2 기반 실시간 시각화 도구

기능:
- 총알을 빨간 구체로 표시
- 드론을 파란 화살표로 표시
- 드론의 속도 벡터를 녹색 화살표로 표시
- 홈 포지션(0,0,-2)을 노란 구체로 표시

사용법:
1. 터미널 1: PX4 + Gazebo 실행
2. 터미널 2: turret_sim_new.py 실행
3. 터미널 3: 이 스크립트 실행
4. 터미널 4: RViz2 실행 후 "/visualization_markers" MarkerArray 추가
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import VehicleOdometry
import numpy as np

class BulletVisualizer(Node):
    def __init__(self):
        super().__init__('bullet_visualizer')
        
        # QoS 설정 (PX4용)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 1. 퍼블리셔: RViz2 Marker
        self.pub_markers = self.create_publisher(
            MarkerArray, 
            '/visualization_markers', 
            10
        )
        
        # 2. 구독자: 총알 데이터
        self.sub_bullets = self.create_subscription(
            Float32MultiArray, 
            '/turret/bullets', 
            self.bullets_cb, 
            10
        )
        
        # 3. 구독자: 드론 위치 (PX4 Odometry)
        self.sub_odom = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_cb,
            qos_profile
        )
        
        # 상태 변수
        self.drone_pos = np.array([0.0, 0.0, 0.0])
        self.drone_vel = np.array([0.0, 0.0, 0.0])
        self.bullets_data = []
        
        # 홈 포지션
        self.home_pos = [0.0, 0.0, -2.0]
        
        # 주기적으로 마커 업데이트 (20Hz)
        self.timer = self.create_timer(0.05, self.publish_markers)
        
        self.get_logger().info('🎨 Bullet & Drone Visualizer Started!')
        self.get_logger().info('📺 Open RViz2 and add MarkerArray topic: /visualization_markers')

    def odom_cb(self, msg):
        """드론 위치/속도 업데이트"""
        self.drone_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.drone_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])

    def bullets_cb(self, msg):
        """총알 데이터 업데이트"""
        data = msg.data
        if len(data) % 6 != 0:
            return
        
        num_bullets = len(data) // 6
        self.bullets_data = []
        
        for i in range(num_bullets):
            idx = i * 6
            pos = [data[idx], data[idx+1], data[idx+2]]
            vel = [data[idx+3], data[idx+4], data[idx+5]]
            self.bullets_data.append((pos, vel))

    def publish_markers(self):
        """모든 마커를 생성하고 퍼블리시"""
        marker_array = MarkerArray()
        
        # 1. 홈 포지션 마커 (노란 구체)
        marker_array.markers.append(self.create_home_marker())
        
        # 2. 드론 마커 (파란 화살표)
        marker_array.markers.append(self.create_drone_marker())
        
        # 3. 드론 속도 벡터 (녹색 화살표)
        marker_array.markers.append(self.create_velocity_marker())
        
        # 4. 총알 마커들 (빨간 구체)
        for i, (pos, vel) in enumerate(self.bullets_data):
            marker_array.markers.append(self.create_bullet_marker(i, pos, vel))
        
        # 5. 사용하지 않는 총알 마커 삭제 (최대 100개까지)
        for i in range(len(self.bullets_data), 100):
            marker_array.markers.append(self.create_delete_marker(i))
        
        self.pub_markers.publish(marker_array)
    
    def create_home_marker(self):
        """홈 포지션 마커 (노란 구체)"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "home"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.home_pos[0]
        marker.pose.position.y = self.home_pos[1]
        marker.pose.position.z = self.home_pos[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # 반투명
        
        return marker
    
    def create_drone_marker(self):
        """드론 마커 (파란 화살표)"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "drone"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 드론 위치
        marker.pose.position.x = float(self.drone_pos[0])
        marker.pose.position.y = float(self.drone_pos[1])
        marker.pose.position.z = float(self.drone_pos[2])
        marker.pose.orientation.w = 1.0
        
        # 크기 (드론 크기)
        marker.scale.x = 0.8  # 길이
        marker.scale.y = 0.1  # 두께
        marker.scale.z = 0.1
        
        # 파란색
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        return marker
    
    def create_velocity_marker(self):
        """드론 속도 벡터 마커 (녹색 화살표)"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "velocity"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 시작점: 드론 위치
        # 끝점: 드론 위치 + 속도 벡터
        marker.points = []
        
        start = Marker().pose.position
        start.x = float(self.drone_pos[0])
        start.y = float(self.drone_pos[1])
        start.z = float(self.drone_pos[2])
        marker.points.append(start)
        
        end = Marker().pose.position
        end.x = float(self.drone_pos[0] + self.drone_vel[0] * 0.5)  # 0.5초 후 위치
        end.y = float(self.drone_pos[1] + self.drone_vel[1] * 0.5)
        end.z = float(self.drone_pos[2] + self.drone_vel[2] * 0.5)
        marker.points.append(end)
        
        # 화살표 스타일
        marker.scale.x = 0.05  # 축 두께
        marker.scale.y = 0.1   # 화살표 머리 두께
        
        # 녹색
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        return marker
    
    def create_bullet_marker(self, idx, pos, vel):
        """총알 마커 (빨간 구체)"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bullets"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = float(pos[2])
        marker.pose.orientation.w = 1.0
        
        # 총알 크기
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # 빨간색
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        return marker
    
    def create_delete_marker(self, idx):
        """사용하지 않는 마커 삭제"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bullets"
        marker.id = idx
        marker.action = Marker.DELETE
        
        return marker


def main():
    rclpy.init()
    node = BulletVisualizer()
    
    print("\n" + "="*60)
    print("🎨 RViz2 Bullet & Drone Visualizer")
    print("="*60)
    print("\n📺 RViz2 설정 방법:")
    print("  1. 터미널에서 'rviz2' 실행")
    print("  2. Add → By topic → /visualization_markers → MarkerArray")
    print("  3. Fixed Frame을 'map'으로 설정")
    print("\n🎨 표시 항목:")
    print("  🔵 파란 화살표: 드론 위치")
    print("  🟢 녹색 화살표: 드론 속도 벡터")
    print("  🔴 빨간 구체: 총알")
    print("  🟡 노란 구체: 홈 포지션 (0,0,-2)")
    print("\n" + "="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n>>> Shutting down visualizer...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()