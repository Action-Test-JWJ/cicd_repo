import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import threading
import yaml
import os


class AudioSenderNode(Node):
    def __init__(self):
        super().__init__('audio_sender_node')
        
        # GStreamer 라이브러리 초기화
        Gst.init(None)
        
        # config 파일 경로 파라미터 받기
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value

        if not config_file:
            config_file = os.path.join(os.path.dirname(__file__), '../config/audio_config.yaml')
            self.get_logger().info(f'Using default config file: {config_file}')

        # yaml 파일 로드
        try:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
                if 'aeirobot_asr' in config_data and 'audio__parameters' in config_data['aeirobot_asr']:
                    self.config = config_data['aeirobot_asr']['audio__parameters']                                               
                else:
                    self.config = {}
                    self.get_logger().error('Invalid config file format')
                    return
                if 'ros__parameters' in config_data['aeirobot_asr']:
                    ros_params = config_data['aeirobot_asr']['ros__parameters']
                    self.config_pub = ros_params.get('pub', {})                        
                else:
                    self.config_pub = {}
                    self.get_logger().error('Invalid config file format')
                    return
                                
            self.get_logger().info(f'Loaded config from: {config_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config file: {str(e)}')
            return

        # Declare parameters with values from config
        self.declare_parameter('device', self.config.get('default_device', 'DeepFilterMic'))
        self.declare_parameter('sample_rate', self.config.get('sample_rate', 48000))
        self.declare_parameter('channels', self.config.get('channels', 1))
        self.declare_parameter('topic', self.config_pub.get('pub_audio', '/aeirobot/audio'))

        # Get parameters
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        topic_name = self.get_parameter('topic').get_parameter_value().string_value

        # Create publisher
        self.publisher = self.create_publisher(Audio, topic_name, 10)

        self.get_logger().info('=== Audio Sender Node ===')
        self.get_logger().info(f'  Device: {self.device}')
        self.get_logger().info(f'  Sample rate: {self.sample_rate}')
        self.get_logger().info(f'  Channels: {self.channels}')
        self.get_logger().info(f'  Topic: {topic_name}')

        # GStreamer
        self.pipeline = None
        self.loop = None
        self.gst_thread = None

        self.init_gstreamer()

    # ======================================================================================================================


    def get_device(self, device=None):
        """장치 연결 테스트"""
        # GStreamer를 사용하여 지정된 오디오 소스(device)가 사용 가능한지 확인
        # 'pulsesrc'는 PulseAudio 소스(주로 리눅스 마이크 입력)를 의미
        # 'fakesink'는 들어오는 데이터를 그냥 버리는 더미 싱크 : 더미싱크인 fakesink를 사용하는 이유 -> 소스가 성공적으로 열리는지만 테스트하는 것이 목적이기 때문
        source = f'pulsesrc device="{device}"' if device else 'pulsesrc'
        try:
            test_pipeline = Gst.parse_launch(f'{source} ! fakesink')
            test_pipeline.set_state(Gst.State.PAUSED)
            ret, _, _ = test_pipeline.get_state(2 * Gst.SECOND)
            test_pipeline.set_state(Gst.State.NULL)
            # SUCCESS 또는 NO_PREROLL 모두 성공
            return ret in (Gst.StateChangeReturn.SUCCESS, Gst.StateChangeReturn.NO_PREROLL)
        except:
            return False

    def init_gstreamer(self):
        """
        오디오를 캡처하고 ROS2 메시지로 변환하기 위한 GStreamer 파이프라인을 설정하고 시작합니다.
        """
        # 1. GStreamer를 사용하여 지정된 오디오 소스(device)가 사용 가능한지 확인
        if self.get_device(self.device):
            source_element = f'pulsesrc device="{self.device}"'
        elif self.get_device():
            source_element = 'pulsesrc'  # 기본 소스 사용
            self.get_logger().warn(f'Device "{self.device}" not found, using default source')
        else:
            self.get_logger().error('No audio device available')
            return

        # 2. GStreamer 파이프라인 구조 정의
        ## Element들은 '!' 문자로 연결
        pipeline_str = (
            f'{source_element} ! '                                                                      # 1. 오디오 소스 (마이크)
            f'audioconvert ! '                                                                          # 2. 오디오 포맷 변환 (호환성 확보)
            f'audioresample ! '                                                                         # 3. 샘플 레이트
            f'audio/x-raw,format=F32LE,rate={self.sample_rate},channels={self.channels} ! '             # 4. 데이터 형식(Caps) 지정: 32비트 부동소수점, 리틀엔디안, 지정된 샘플 레이트/채널
            f'appsink name=sink emit-signals=true sync=false max-buffers=1 drop=false blocksize=5760'   # 5. 앱 싱크: GStreamer 파이프라인의 데이터를 현재 파이썬 코드로 가져오는 역할
        )

        self.get_logger().info(f'Pipeline: {pipeline_str}')

        # 3. 문자열로 정의된 파이프라인 구조를 실제 GStreamer 파이프라인 객체로 생성
        self.pipeline = Gst.parse_launch(pipeline_str)
        if not self.pipeline:
            self.get_logger().error('Failed to create pipeline')
            return

        # 4. GStreamer와 파이썬 코드 연결
        appsink = self.pipeline.get_by_name('sink')                 # 파이프라인에서 'sink'라는 이름의 appsink element 가져오기 
        appsink.connect('new-sample', self.on_new_sample)           # appsink에 'new-sample' 신호가 발생할 때마다 self.on_new_sample 함수가 호출되도록 연결

        # 5. 파이프라인을 'PLAYING' 상태로 만들어 데이터 흐름을 시작
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error('Failed to start pipeline')
            return

        # 6. GStreamer의 내부 이벤트 루프(GLib.MainLoop)를 별도의 스레드에서 실행 : 이렇게 해야 GStreamer 이벤트 처리가 ROS2의 메인 스레드를 막지 않음
        self.loop = GLib.MainLoop()
        self.gst_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.gst_thread.start()

        self.get_logger().info('GStreamer pipeline started')

    def on_new_sample(self, sink):
        """
        GStreamer appsink로부터 새로운 오디오 샘플이 도착했을 때 호출되는 콜백 함수.
        """
        # 1. appsink로부터 새로운 샘플(데이터 청크)을 가져옴
        sample = sink.emit('pull-sample')
        if not sample:
            return Gst.FlowReturn.ERROR

        # 2. 샘플에서 실제 데이터가 담긴 버퍼를 추출합니다.
        buffer = sample.get_buffer()
        # 3. 버퍼의 메모리에 안전하게 접근하기 위해 'map'합니다. READ 플래그는 읽기 전용 접근을 의미합니다.
        success, map_info = buffer.map(Gst.MapFlags.READ)

        if success:
            # 4. ROS2 Audio 메시지를 생성합니다.
            msg = Audio()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'microphone'
            msg.sample_rate = self.sample_rate
            msg.channels = self.channels
            msg.encoding = 'F32LE'
            # print(type(map_info.data)) : <class 'bytes'> 임 이미
            msg.data = map_info.data          # list 변환 오버헤드 제거: bytes로 직접 할당 (rclpy 최적화)

            # 5. 생성된 메시지를 토픽으로 발행
            self.publisher.publish(msg)
            
            # 6. [매우 중요] 버퍼 사용이 끝났으므로 'unmap'하여 리소스를 해제
            buffer.unmap(map_info)

        return Gst.FlowReturn.OK

    def destroy_node(self):
        # 노드가 종료될 때 GStreamer 리소스를 안전하게 해제
        if self.loop:
            self.loop.quit()
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        self.get_logger().info('Audio Sender Node destroyed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioSenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
