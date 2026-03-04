#!/usr/bin/env python3
"""
ROS2 Audio 토픽을 구독하여 VAD 및 음성 세그멘테이션을 수행하는 노드

sender_node.py가 발행하는 Audio 토픽을 구독하여:
1. 에너지 레벨(dB) 계산
2. VAD (Voice Activity Detection) 수행
3. 음성 구간을 WAV 파일로 저장
4. ASR 상태 및 파일 경로 발행
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import UInt8MultiArray
import threading
import yaml
import os
import time
import sys
from std_msgs.msg import String, Bool
from aeirobot_asr.processors.audio_processor import AudioProcessor
from aeirobot_asr.processors.energy_processor import EnergyProcessor
from aeirobot_asr.processors.vad_engines.vad_factory import VADFactory
from aeirobot_asr.processors.speech_processor import SpeechProcessor
import numpy as np
import traceback


def clear_terminal():
    """터미널 화면을 클리어합니다."""
    os.system('clear')


def move_cursor_up(lines):
    """터미널 화면 지정된 줄 수만큼 위로 이동합니다."""
    sys.stdout.write(f"\033[{lines}A")
    sys.stdout.flush()


class ASRNode(Node):
    """ROS2 Audio 토픽을 구독하여 ASR을 수행하는 노드"""
    
    def __init__(self):
        super().__init__('asr_ros_node')

        try:
            # 콜백 그룹 생성
            # self.service_group = MutuallyExclusiveCallbackGroup()         # 단 하나의 Callback만을 실행 (리소스에 대한 상호 배제 조건이 필요한 콜백만)   # 현재는 안쓰지만 나중에 service 또는 action 사용 추가 
            self.subscription_group = ReentrantCallbackGroup()              # Callbacks가 제한 없이 동시에 병렬로 실행

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
                        self.config_sub = ros_params.get('sub', {})
                    else:
                        self.config_pub = {}
                        self.config_sub = {}
                        self.get_logger().error('Invalid config file format')
                        return

                self.get_logger().info(f'Loaded config from: {config_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load config file: {str(e)}')
                return

            # 파라미터 선언
            self.declare_parameter('pub_asr_status', self.config_pub.get('pub_asr_status', '/aeirobot/asr/status'))
            self.declare_parameter('pub_asr_file', self.config_pub.get('pub_asr_file', '/aeirobot/asr/file_path'))
            self.declare_parameter('pub_asr_stream', self.config_pub.get('pub_asr_stream', ''))
            self.declare_parameter('sub_audio', self.config_sub.get('sub_audio', '/edie/audio/df_data'))
            self.declare_parameter('sample_rate', self.config.get('sample_rate', 48000))
            self.declare_parameter('encoding', self.config.get('encoding', 'F32LE'))

            # 파라미터 가져오기
            self.asr_status_topic = self.get_parameter('pub_asr_status').value
            self.asr_file_topic = self.get_parameter('pub_asr_file').value
            self.asr_stream_topic = self.get_parameter('pub_asr_stream').value
            self.audio_topic = self.get_parameter('sub_audio').value
            self.sample_rate = self.get_parameter('sample_rate').value
            self.encoding = self.get_parameter('encoding').value
            
            # 스트림 모드 여부 (pub_asr_stream이 설정되어 있으면 스트림 모드)
            self.use_stream_mode = bool(self.asr_stream_topic)
            
            # 상태 변수 초기화
            self.processing = True
            self.latest_vad = 0.0
            self.latest_db = -80.0
            self.is_speaking = False
            self._prev_is_speaking = False
            self.last_event = "대기중..."
            self.last_saved_file = "(없음)"
            self._sample_rate_synced = False

            # processor들 초기화
            self._initialize_processors()

            # Create publisher
            self.asr_status_pub = self.create_publisher(Bool, self.asr_status_topic, 10)
            self.asr_file_pub = self.create_publisher(String, self.asr_file_topic, 10)
            
            # 스트림 모드일 경우 UInt8MultiArray 타입 publisher 추가 생성
            if self.use_stream_mode:
                self.asr_stream_pub = self.create_publisher(UInt8MultiArray, self.asr_stream_topic, 10)
                self.get_logger().info(f'Created publishers: {self.asr_status_topic}, {self.asr_stream_topic} (STREAM MODE)')
            else:
                self.asr_stream_pub = None
                self.get_logger().info(f'Created publishers: {self.asr_status_topic}, {self.asr_file_topic} (FILE MODE)')

            # QoS profile (match with audio publisher)
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            # Create subscriper
            self.subscription = self.create_subscription(
                UInt8MultiArray,
                self.audio_topic,
                self.audio_callback,
                qos_profile,
                callback_group=self.subscription_group
            )
            self.get_logger().info(f'Subscribed to: {self.audio_topic}')

            # 종료 처리를 위한 이벤트
            self.shutdown_event = threading.Event()

            # 실시간 디스플레이용 상태 변수
            self.display_lock = threading.Lock()

            # 실시간 디스플레이 스레드 시작
            self.display_thread = threading.Thread(target=self.update_display, daemon=True)
            self.display_thread.start()

            self.get_logger().info('ASR ROS Node initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Error initializing ASR ROS node: {str(e)}')
            raise

    # ===============================================================================================================================================

    def _initialize_processors(self):
        """프로세서 초기화"""
        try:
            # 오디오 디바이스 매니저 초기화 (WAV 저장용으로 필요)
            self.audio_manager = AudioProcessor(self)

            # 에너지 모니터 초기화
            self.energy_monitor = EnergyProcessor(self)

            # VAD 프로세서 초기화 
            self.vad_processor = VADFactory.create_vad(self)

            # 음성 프로세서 초기화
            self.speech_processor = SpeechProcessor(
                self,
                self.vad_processor,
                self.energy_monitor,
                self.audio_manager,
                use_stream_mode=self.use_stream_mode
            )

            self.get_logger().info('All processors initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Error initializing processors: {str(e)}')
            raise

    def audio_callback(self, msg):
        """ROS2 오디오 메시지 콜백"""
        # 첫 메시지에서 샘플레이트 동기화
        if not self._sample_rate_synced:
            self.speech_processor.sample_rate = self.sample_rate
            self._sample_rate_synced = True
            self.get_logger().info(f'Sample rate synced to: {self.sample_rate}')

        try:
            # ROS2 오디오 메시지 처리
            audio_data = bytes(msg.data)
            result, db_level, vad_score = self.speech_processor.process_frame(audio_data, self.encoding)

            # 현재 음성 감지 상태
            current_is_speaking = self.speech_processor.is_speaking

            # 디스플레이용 상태 업데이트
            with self.display_lock:
                self.latest_db = db_level
                self.latest_vad = vad_score
                self.is_speaking = current_is_speaking

            # 음성인식 시작 감지 (False → True)
            if current_is_speaking and not self._prev_is_speaking:
                status_msg = Bool()
                status_msg.data = True
                self.asr_status_pub.publish(status_msg)
                # self.get_logger().info('ASR started - published True')

            # VAD로 음성 감지 완료 시 처리
            if result:
                if self.use_stream_mode:
                    # 스트림 모드: 오디오 데이터 직접 발행
                    is_final, audio_data_result = result
                    if is_final and audio_data_result is not None:
                        speech_segment, sample_rate = audio_data_result
                        
                        # 음성인식 종료 - False 발행
                        status_msg = Bool()
                        status_msg.data = False
                        self.asr_status_pub.publish(status_msg)
                        # self.get_logger().info('ASR ended - published False')

                        # UInt8MultiArray 메시지 생성 및 발행
                        audio_msg = UInt8MultiArray()
                        audio_msg.data = list(speech_segment.tobytes())

                        self.asr_stream_pub.publish(audio_msg)
                        # self.get_logger().info(f'Published audio stream: {len(speech_segment)} samples ({len(speech_segment)/sample_rate:.2f}s)')

                        with self.display_lock:
                            self.last_event = "스트림 발행됨"
                            self.last_saved_file = f"{len(speech_segment)/sample_rate:.2f}s audio"
                else:
                    # 파일 모드: 기존 방식 (wav 파일 저장 후 경로 발행)
                    is_final, wav_path = result
                    if is_final and wav_path:
                        # 음성인식 종료 - False 발행
                        status_msg = Bool()
                        status_msg.data = False
                        self.asr_status_pub.publish(status_msg)
                        # self.get_logger().info('ASR ended - published False')

                        # 파일 경로 발행
                        file_msg = String()
                        file_msg.data = wav_path
                        self.asr_file_pub.publish(file_msg)
                        # self.get_logger().info(f'Published file path: {wav_path}')

                        with self.display_lock:
                            self.last_event = "음성 저장됨"
                            self.last_saved_file = wav_path

            # 이전 상태 업데이트
            self._prev_is_speaking = current_is_speaking

        except Exception as e:
            self.get_logger().error(f'Error in audio processing: {str(e)}')
            traceback.print_exc()

    def update_display(self):
        """실시간 디스플레이 업데이트 스레드"""
        last_update = time.time()
        update_interval = 0.05  # ~20Hz
        bar_width = 30
        display_lines = 6
        first_update = True

        while not self.shutdown_event.is_set():
            try:
                current_time = time.time()
                if current_time - last_update >= update_interval:
                    with self.display_lock:
                        vad = self.latest_vad
                        db = self.latest_db
                        is_speaking = self.is_speaking
                        last_event = self.last_event
                        last_saved = self.last_saved_file

                    # VAD 바
                    vad_blocks = int(vad * bar_width)
                    vad_bar = '█' * vad_blocks + '░' * (bar_width - vad_blocks)

                    # DB 바 (-80dB ~ 0dB 범위를 0~1로)
                    normalized_db = min(1.0, max(0.0, (db + 80) / 80))
                    db_blocks = int(normalized_db * bar_width)
                    db_bar = '█' * db_blocks + '░' * (bar_width - db_blocks)

                    # 상태 이모지
                    status = "🗣️ 음성감지" if is_speaking else "🔇 대기중"

                    if first_update:
                        clear_terminal()
                        first_update = False
                    else:
                        move_cursor_up(display_lines)

                    # 디스플레이 출력
                    print("\033[K" + "=" * 60)
                    print(f"\033[KAudio Level: [{db_bar}] {db:>6.1f} dB")
                    print(f"\033[KVAD Level:   [{vad_bar}] {vad:.2f}")
                    print(f"\033[KStatus: {status} | Event: {last_event}")
                    print(f"\033[KSaved: {last_saved}")
                    print("\033[K" + "=" * 60)

                    last_update = current_time
                else:
                    time.sleep(0.01)
            except Exception as e:
                if not self.shutdown_event.is_set():
                    pass  # 디스플레이 오류는 조용히 무시
                break

    def destroy_node(self):
        """노드 종료"""
        self.shutdown_event.set()
        self.get_logger().info('ASR ROS Node destroyed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    asr_node = ASRNode()

    # MultiThreadedExecutor 생성 및 실행
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(asr_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        asr_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
