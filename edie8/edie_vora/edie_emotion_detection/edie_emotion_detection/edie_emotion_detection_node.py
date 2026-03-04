#!venv/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Bool
from sensor_msgs.msg import CompressedImage, Image 
import sys, os, cv2, errno, time
import numpy as np
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from demo import Demo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

class PubEmotionNode(Node):
    def __init__(self):
        super().__init__('edie_emotion_result_pub')
        self.bridge = CvBridge()

        # Demo 클래스 인스턴스 생성
        package_share_directory = get_package_share_directory('edie_emotion_detection')
        model_path = os.path.join(package_share_directory, 'model', '2803_vgg16_full.model')     
        self.demo = Demo(model_path, show_window=False)
        
        # ===== QoS 설정 =====
        # (1) voice_heard: 이벤트 유실 방지용
        voice_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 퍼블리셔가 transient_local이면 늦게 붙어도 수신
            history=HistoryPolicy.KEEP_LAST
        )
        # (2) result_qos: 1회성 이미지 전달, 늦게 구독해도 전달 / 오래된 건 폐기
        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # STT가 늦게 붙어도 최근 1장 전달
            history=HistoryPolicy.KEEP_LAST,
            lifespan=Duration(seconds=5)  # 너무 오래된 이미지가 재전송되지 않도록
        )
        # 저지연 상태 퍼블리셔용 QoS (backpressure 최소화)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publisher
        self.pub_emotion_array = self.create_publisher(Float64MultiArray, '/edie8/vision/normalized_emotion_depth_array', result_qos)     
        self.pub_emption_state = self.create_publisher(String, '/edie8/vision/emotion_state', state_qos)
        self.pub_emotion_result_image  = self.create_publisher(Image, '/edie8/vision/face_img', result_qos)

        # Subscriberce
        self.sub_img = self.create_subscription(Image, '/edie8/vision/image_raw', self.ImageCallback, 10)
        self.sub_voice = self.create_subscription(Bool, '/edie8/llm/voice_heard', self.VoiceCallback, voice_qos)

        self.emotion_data = np.array([0.0]*7, dtype=np.float64) 
        self.oneshot_flag = False  # voice_heard 트리거 후 1회 발행 플래그
        self.last_image_header = None   # 원본 이미지 헤더 보관

        # 프로파일 로그 on/off (true면 각 인퍼런스마다 단계별 소요시간(ms) 로그)
        self.declare_parameter('profile_infer', True)
        self._prof_enabled = bool(self.get_parameter('profile_infer').value)
        self.get_logger().info(f'[_init__] profile_infer: {self._prof_enabled}')

    def ImageCallback(self, msg: Image):   
        self.last_image_header = msg.header  # stamp, frame_id 같이 보관
        # 프로파일 시작 시간
        t_init = time.perf_counter() if self._prof_enabled else 0.0
        t_cv = 0.0
  
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self._prof_enabled:
                t_cv = time.perf_counter()
        except Exception as e:
            self.get_logger().error(f"cv_bridge convert failed: {e}")
            return
        
        result = self.demo.ProcessFrame(frame, self._prof_enabled, t_init, t_cv)
        if result is None:
            return

        label, emotion_data, vis_frame = result

        self.PubEmotionState(label)
        self.PubEmotionArray(emotion_data)
        if self.oneshot_flag:
            self.PubResultImage(vis_frame)

    def VoiceCallback(self, msg: Bool):
        if msg.data:  # True일 때만 발행 대기 상태
            if not self.oneshot_flag:
                self.oneshot_flag = True
                self.get_logger().info("voice_heard=True 수신 → face_img 원샷 발행 대기 상태로 전환")
        # False는 무시(필요하면 여기서 비무장/재무장 정책을 넣을 수 있음)

    def PubEmotionState(self, emotion_state):
        msg = String()
        msg.data = emotion_state
        self.pub_emption_state.publish(msg)

    def PubEmotionArray(self, emotion_data):
        msg = Float64MultiArray()
        msg.data = emotion_data.tolist()
        self.pub_emotion_array.publish(msg)                                                                    
        
    def PubResultImage(self, frame):
        fra = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # fra = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
        if self.last_image_header is not None:
            fra.header = self.last_image_header  # 카메라 좌표계 및 타임스탬프 유지

            self.pub_emotion_result_image.publish(fra)
            self.oneshot_flag = False
            self.get_logger().info("face_img 1회 발행 완료 (원샷)")

def main(argv=None):
    if argv is None:     # new   
        argv = sys.argv  # argv가 None이면 sys.argv로 설정
    
    rclpy.init(args=argv)
    node = PubEmotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 
