import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

def pitch_to_quat(pitch: float):
    half = pitch * 0.5
    return (0.0, math.sin(half), 0.0, math.cos(half))   # x,y,z,w

class MultiBoxPublisher(Node):
    def __init__(self):
        super().__init__('multi_box_publisher')
        self.pub_  = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_boxes)

        # 선분 정의:  (x, y, z)   ※ y 는 모두 0
        self.segments = [
            ((-1.675, 0.0, 0.0), (0.83, 0.0, 0.0)),  # 아래 가로
            ((-1.675, 0.0, 0.0), (-1.675, 0.0, 3.065)),  # 오른쪽 세로
            ((-1.675, 0.0, 3.065), (0.83, 0.0, 3.065)),  # 왼쪽 세로
            (( 0.83, 0.0, 0.0), (0.83, 0.0, 3.065)),  # 위   가로
        ]

        self.thickness = 0.05   # y축 두께
        self.height    = 0.20   # z축 높이 (벽의 위로 튀어나온 정도)

    def publish_boxes(self):
        for idx, (start, end) in enumerate(self.segments):
            dx = end[0] - start[0]
            dz = end[2] - start[2]
            length = math.hypot(dx, dz)               # √(dx²+dz²)
            mid_x  = (start[0] + end[0]) * 0.5
            mid_z  = (start[2] + end[2]) * 0.5
            pitch  = math.atan2(dz, dx)               # y축 회전
            qx, qy, qz, qw = pitch_to_quat(pitch)

            m = Marker()
            m.header.frame_id = 'marker_0'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns, m.id        = 'aruco_wall', idx
            m.type, m.action  = Marker.CUBE, Marker.ADD

            m.pose.position.x = mid_x
            m.pose.position.y = 0.0                   # xz 평면 중앙
            m.pose.position.z = mid_z + self.height * 0.5

            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw

            m.scale.x = length          # 선분 길이
            m.scale.y = self.height     # 벽 높이 
            m.scale.z = self.thickness     # 벽 두께 

            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.7, 1.0, 0.5
            self.pub_.publish(m)

def main():
    rclpy.init()
    node = MultiBoxPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
