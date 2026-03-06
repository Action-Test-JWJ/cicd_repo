#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import yaml
import os
import time
import numpy as np
from std_msgs.msg import String, Bool
from edie_msgs.msg import Audio
from aeirobot_stt.processors.stt_processor import STTProcessor
import traceback


class AeirobotSTT(Node):
    def __init__(self):
        super().__init__('aeirobot_stt')

        try:
            # 콜백 그룹 생성
            # self.service_group = MutuallyExclusiveCallbackGroup()         # 단 하나의 Callback만을 실행 (리소스에 대한 상호 배제 조건이 필요한 콜백만)   # 현재는 안쓰지만 나중에 service 또는 action 사용 추가 
            self.subscription_group = ReentrantCallbackGroup()              # Callbacks가 제한 없이 동시에 병렬로 실행

            # config 파일 경로 파라미터 받기
            self.declare_parameter('config_file', '')
            config_file = self.get_parameter('config_file').value

            if not config_file:
                config_file = os.path.join(os.path.dirname(__file__), '../config/stt_config.yaml')
                self.get_logger().info(f'Using default config file: {config_file}')

            # yaml 파일 로드
            try:
                with open(config_file, 'r') as f:
                    config_data = yaml.safe_load(f)
                    if 'aeirobot_stt' in config_data:
                        # ros__parameters 로드
                        if 'ros__parameters' in config_data['aeirobot_stt']:
                            ros_params = config_data['aeirobot_stt']['ros__parameters']
                            self.config_pub = ros_params.get('pub', {})
                            self.config_sub = ros_params.get('sub', {})
                        else:
                            self.config_pub = {}
                            self.config_sub = {}
                            self.get_logger().error('Invalid config file format: ros__parameters not found')
                            return
                        
                        # stt__parameters 로드 (STTProcessor용)
                        if 'stt__parameters' in config_data['aeirobot_stt']:
                            self.config = config_data['aeirobot_stt']['stt__parameters']
                        else:
                            self.config = {}
                            self.get_logger().warn('stt__parameters not found, using defaults')
                    else:
                        self.config_pub = {}
                        self.config_sub = {}
                        self.config = {}
                        self.get_logger().error('Invalid config file format')
                        return
                self.get_logger().info(f'Loaded config from: {config_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load config file: {str(e)}')
                return

            # 설정 파라미터 선언 (ROS2 파라미터로 오버라이드 가능한 것들만)
            self.declare_parameter('pub_stt_result', self.config_pub.get('pub_stt_result', '/aeirobot/stt/result'))
            self.declare_parameter('sub_speaker_status', self.config_sub.get('sub_speaker_status', '/aeirobot/asr/status'))
            self.declare_parameter('sub_wav_path', self.config_sub.get('sub_wav_path', '/aeirobot/asr/file'))
            self.declare_parameter('sub_asr_stream', self.config_sub.get('sub_asr_stream', ''))

            # 파라미터 가져오기
            self.pub_stt_result = self.get_parameter('pub_stt_result').value
            self.sub_speak_status = self.get_parameter('sub_speaker_status').value
            self.sub_wav_path = self.get_parameter('sub_wav_path').value
            self.sub_asr_stream = self.get_parameter('sub_asr_stream').value
            
            # 스트림 모드 여부 (sub_asr_stream이 설정되어 있으면 스트림 모드)
            self.use_stream_mode = bool(self.sub_asr_stream)

            # STTProcessor 초기화
            self.stt_processor = STTProcessor(self)
            self.get_logger().info('STTProcessor initialized')

            # 발행자 생성
            self.stt_result_publisher = self.create_publisher(String, self.pub_stt_result, 10)

            # 구독자 생성
            ## asr 인식 상태 구독
            self.speak_status_subscriber = self.create_subscription(
                Bool,
                self.sub_speak_status,
                self.speak_status_callback,
                10,
                callback_group=self.subscription_group
            )
            ## stt 처리를 위한 음성 파일 위치 구독 (파일 모드)
            self.wav_path_subscriber = self.create_subscription(
                String,
                self.sub_wav_path,
                self.wav_path_callback,
                10,
                callback_group=self.subscription_group
            )
            
            ## 오디오 스트림 구독 (스트림 모드)
            if self.use_stream_mode:
                self.audio_stream_subscriber = self.create_subscription(
                    Audio,
                    self.sub_asr_stream,
                    self.audio_stream_callback,
                    10,
                    callback_group=self.subscription_group
                )
                self.get_logger().info(f'Subscribed to audio stream: {self.sub_asr_stream} (STREAM MODE)')
            else:
                self.audio_stream_subscriber = None
                self.get_logger().info('Stream mode disabled (FILE MODE)')
            
            # 상태 변수 초기화
            self.speak_status = False    # 발화 상태 (True = 말하는 중, False = 말 끝남)
            self.stt_processing = False  # STT 처리 플래그
            self.wav_path = ""           # 처리할 WAV 파일 경로

            self.get_logger().info('STT node initialized successfully')
            if self.use_stream_mode:
                self.get_logger().info(f'Subscribed to: {self.sub_speak_status}, {self.sub_asr_stream} (STREAM MODE)')
            else:
                self.get_logger().info(f'Subscribed to: {self.sub_speak_status}, {self.sub_wav_path} (FILE MODE)')
            self.get_logger().info(f'Publishing to: {self.pub_stt_result}')

        except Exception as e:
            self.get_logger().error(f'Error initializing STT node: {str(e)}')
            traceback.print_exc()
            raise

    def speak_status_callback(self, msg):
        """발화 상태 콜백 - True에서 False로 바뀔 때 STT 처리 트리거"""
        prev_status = self.speak_status
        self.speak_status = msg.data
        self.get_logger().info(f"speak_status: {prev_status} -> {msg.data}")
        
        # True → False 전환 감지 (말이 끝남) = stt 진행 중 
        if prev_status == True and msg.data == False:
            self.get_logger().info("Speech ended detected, triggering STT processing")
            self.stt_processing = True
            self.process_stt()

    def wav_path_callback(self, msg):
        """WAV 파일 경로 콜백"""
        self.get_logger().info(f"wav_path received: {msg.data}")
        self.wav_path = msg.data

    def audio_stream_callback(self, msg: Audio):
        """오디오 스트림 콜백 - 스트림 모드에서 사용"""
        try:
            self.get_logger().info(f"Audio stream received: {msg.frames} frames, {msg.sample_rate} Hz")
            start_time = time.time()
            
            # Audio 메시지에서 오디오 데이터 추출
            if msg.encoding.upper() == 'F32LE':
                audio_data = np.frombuffer(bytes(msg.data), dtype=np.float32)
            else:
                # S16LE 등 다른 인코딩의 경우
                int16_data = np.frombuffer(bytes(msg.data), dtype=np.int16)
                audio_data = int16_data.astype(np.float32) / 32768.0
            
            # STT 처리 (스트림 모드)
            result = self.stt_processor.process_audio_stream(audio_data, msg.sample_rate)
            
            processing_time = time.time() - start_time
            
            if result:
                text, lang, confidence = result
                if text:
                    # 결과 발행
                    result_msg = String()
                    result_msg.data = text
                    self.stt_result_publisher.publish(result_msg)
                    self.get_logger().info(f"STT stream result published: '{text}' (lang: {lang}, conf: {confidence:.2f}, time: {processing_time:.2f}s)")
                else:
                    self.get_logger().warn("STT stream returned empty text")
            else:
                self.get_logger().warn("STT stream processing returned None")
                
        except Exception as e:
            self.get_logger().error(f"Error in audio_stream_callback: {str(e)}")
            traceback.print_exc()

    def process_stt(self):
        """STT 처리 실행"""
        if not self.stt_processing:
            return
            
        if not self.wav_path:
            self.get_logger().warn("No wav_path available for STT processing")
            self.stt_processing = False
            return
            
        try:
            self.get_logger().info(f"Processing STT for: {self.wav_path}")
            start_time = time.time()
            
            # STT 처리
            result = self.stt_processor.process_speech_to_text(self.wav_path)
            
            processing_time = time.time() - start_time
            
            if result:
                text, lang, confidence = result
                if text:
                    # 결과 발행
                    result_msg = String()
                    result_msg.data = text
                    self.stt_result_publisher.publish(result_msg)
                    self.get_logger().info(f"STT result published: '{text}' (lang: {lang}, conf: {confidence:.2f}, time: {processing_time:.2f}s)")
                else:
                    self.get_logger().warn("STT returned empty text")
            else:
                self.get_logger().warn("STT processing returned None")
                
        except Exception as e:
            self.get_logger().error(f"Error in process_stt: {str(e)}")
            traceback.print_exc()
        finally:
            # 처리 완료 후 플래그 리셋
            self.stt_processing = False
            self.get_logger().debug("STT processing flag reset to False")


def main(args=None):
    rclpy.init(args=args)

    stt_node = AeirobotSTT()

    # MultiThreadedExecutor 생성 및 실행
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(stt_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stt_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
