#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import RegionOfInterest, Image
from cv_bridge import CvBridge
from demo import Demo

class PubHumanCenterPointNode(Node):
    def __init__(self):
        super().__init__('edie_human_result_pub')
        # 클래스 인스턴스 변수
        self.bridge = CvBridge()
        self.demo = Demo(show_window=False)  # 필요 없으면 False
        # Publisher
        self.pub_human_center_point = self.create_publisher(Point, '/edie8/vision/closest_human_point', 10)
        self.pub_human_roi = self.create_publisher(RegionOfInterest, '/edie8/vision/closest_human_roi', 10)
        # Subscriber
        self.sub_img = self.create_subscription(Image, '/edie8/vision/image_raw', self.ImageCallback, 10)

    def ImageCallback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge convert failed: {e}")
            return

        result = self.demo.ProcessFrame(frame)
        if result is None:
            return

        x, y, w, h = result
        self.PubHumanCenterPoint(x, y)
        self.PubHumanROI(x, y, w, h)

    def PubHumanCenterPoint(self, x, y):
        msg = Point()
        msg.x, msg.y, msg.z = float(x), float(y), 0.0

        self.pub_human_center_point.publish(msg)

    def PubHumanROI(self, x, y, w, h):
        msg = RegionOfInterest()
        msg.x_offset, msg.y_offset = int(x), int(y)
        msg.width, msg.height = int(w), int(h)
        msg.do_rectify = False
		
        self.pub_human_roi.publish(msg)

def main(argv=None):
    if argv is None:     # new   
        argv = sys.argv  # argv가 None이면 sys.argv로 설정

    rclpy.init(args=argv)
    node = PubHumanCenterPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()