#!/usr/bin/env python3
"""
ROS2 Audio 토픽을 구독하여 5초간 녹음 후 WAV 파일로 저장하는 테스트 노드

사용법:
1. sender_node 실행: ros2 run edie_mic sender_node
2. 녹음 노드 실행: ros2 run aeirobot_asr record
3. 엔터 누르면 5초 녹음 시작
4. ~/.aeirobot_asr/test_recording.wav로 저장
"""

import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
import numpy as np
import soundfile as sf
import os
import time
import threading
import yaml

from aeirobot_asr.processors.energy_processor import EnergyProcessor


class AudioRecordNode(Node):
    """ROS2 Audio 토픽을 구독하여 녹음하는 테스트 노드"""
    
    def __init__(self):
        super().__init__('audio_record_node')
        
        # config 파일 경로 파라미터 받기
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value

        if not config_file:
            config_file = os.path.join(os.path.dirname(__file__), '../config/audio_config.yaml')

        # yaml 파일 로드
        try:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
                if 'ros__parameters' in config_data.get('aeirobot_asr', {}):
                    ros_params = config_data['aeirobot_asr']['ros__parameters']
                    self.config_sub = ros_params.get('sub', {})
                else:
                    self.config_sub = {}
        except Exception as e:
            self.get_logger().warn(f'Failed to load config: {str(e)}, using defaults')
            self.config_sub = {}
        
        # 파라미터
        self.declare_parameter('sub_audio', self.config_sub.get('sub_audio', '/aeirobot/edie/audio'))
        self.audio_topic = self.get_parameter('sub_audio').value
        
        # 녹음 설정
        self.recording = False
        self.record_duration = 5.0  # 5초
        self.audio_buffer = []
        self.sample_rate = None
        self.channels = None
        self.record_start_time = None
        
        self.energy_monitor = EnergyProcessor(self)
        
        # 구독자 생성
        self.subscription = self.create_subscription(
            Audio,
            self.audio_topic,
            self.audio_callback,
            10
        )
        
        self.get_logger().info('=== Audio Record Node ===')
        self.get_logger().info(f'  Topic: {self.audio_topic}')
        self.get_logger().info('  Press Enter to start 5-second recording...')
        
        # 키 입력 스레드
        self.input_thread = threading.Thread(target=self.wait_for_input, daemon=True)
        self.input_thread.start()
    
    def wait_for_input(self):
        """엔터 키 입력 대기"""
        while True:
            try:
                input("\n>>> 엔터를 누르면 5초 녹음을 시작합니다...\n")
                self.start_recording()
            except EOFError:
                break
    
    def start_recording(self):
        """녹음 시작"""
        if self.recording:
            self.get_logger().warn('Already recording!')
            return
        
        self.audio_buffer = []
        self.recording = True
        self.record_start_time = time.time()
        self.get_logger().info('🔴 Recording started...')
    
    def audio_callback(self, msg):
        """오디오 콜백"""
        # 샘플레이트 및 인코딩 저장
        if self.sample_rate is None:
            self.sample_rate = msg.sample_rate
            self.channels = msg.channels
            self.encoding = msg.encoding
            self.get_logger().info(
                f'Audio format: {self.sample_rate}Hz, {self.channels}ch, {msg.encoding}'
            )
        
        if not self.recording:
            return
        
        # 녹음 시간 체크
        elapsed = time.time() - self.record_start_time
        if elapsed >= self.record_duration:
            self.stop_recording()
            return
        
        # raw bytes 데이터를 그대로 저장
        data = bytes(msg.data)
        self.audio_buffer.append(data)
        
        # 진행 상황 출력
        remaining = self.record_duration - elapsed
        print(f'\r🔴 Recording... {remaining:.1f}s remaining', end='', flush=True)
    
    def stop_recording(self):
        """녹음 중지 및 파일 저장"""
        self.recording = False
        print()  # 줄바꿈
        self.get_logger().info('⏹ Recording stopped')
        
        if not self.audio_buffer:
            self.get_logger().error('No audio data recorded!')
            return
        
        # 모든 버퍼를 하나로 합치기
        all_data = b''.join(self.audio_buffer)

        # 인코딩에 따라 bytes → numpy 배열 변환
        enc = (self.encoding or 'S16LE').upper()
        if enc == 'F32LE':
            # F32LE bytes → float32 numpy array
            audio_array = np.frombuffer(all_data, dtype=np.float32)
        else:
            # 기본: S16LE bytes → int16 → float32(-1~1)로 정규화
            int16_array = np.frombuffer(all_data, dtype=np.int16)
            audio_array = int16_array.astype(np.float32) / 32768.0

        # WAV 파일 저장
        output_dir = os.path.join(os.path.expanduser('~'), '.aeirobot_asr')
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, 'test_recording.wav')

        sf.write(output_path, audio_array, self.sample_rate)

        duration = len(audio_array) / self.sample_rate
        self.get_logger().info(f'✅ Saved: {output_path}')
        self.get_logger().info(f'   Duration: {duration:.2f}s, Samples: {len(audio_array)}')
        
        # 버퍼 초기화
        self.audio_buffer = []
        
        print('\n>>> 엔터를 누르면 다시 녹음합니다...\n')


def main(args=None):
    rclpy.init(args=args)
    node = AudioRecordNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
