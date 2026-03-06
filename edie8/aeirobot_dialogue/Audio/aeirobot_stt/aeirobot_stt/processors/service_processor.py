#!/usr/bin/env python3
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String

class ServiceProcessor:
    def __init__(self, node: Node, config: dict):
        self.node = node
        self.config = config
        
        # 설정값
        self.silence_duration = config.get('service', {}).get('silence_duration', 1.5)  # 기본값 1.5초
        
        # 상태 변수
        self.active = False
        self.is_speaking = False  # 현재 말하는 중인지
        self.silence_start_time = None  # 말이 끝난 시점
        self.audio_buffer = []  # 전체 오디오 버퍼

    def start_service(self):
        """서비스 시작"""
        self.active = True
        self.is_speaking = False
        self.silence_start_time = None
        self.audio_buffer = []
        self.node.get_logger().info('Service started')

    def process_speech(self, speech_data):
        """음성 데이터 처리"""
        if not self.active:
            return False

        # 오디오 버퍼에 추가
        self.audio_buffer.append(speech_data)
        current_time = time.time()

        # 말하기 시작
        if not self.is_speaking:
            self.is_speaking = True
            self.silence_start_time = None
            self.node.get_logger().info('Speech detected')
            return False

        return False

    def end_speech(self, current_time=None):
        """발화 종료 처리"""
        if not current_time:
            current_time = time.time()

        if self.is_speaking:
            self.is_speaking = False
            self.silence_start_time = current_time
            self.node.get_logger().info('Speech ended, starting silence check')

    def check_silence(self):
        """묵음 시간 체크"""
        if not self.active:
            return False

        # 말하는 중이면 체크하지 않음
        if self.is_speaking:
            return False

        # 아직 말이 시작되지 않았거나 끝나지 않았으면 체크하지 않음
        if self.silence_start_time is None:
            return False

        current_time = time.time()
        silence_duration = current_time - self.silence_start_time

        # 설정된 묵음 시간보다 길면 서비스 종료
        if silence_duration >= self.silence_duration:
            self.node.get_logger().info(f'Silence duration reached: {silence_duration:.2f}s')
            return True

        return False

    def reset(self):
        """상태 초기화"""
        self.active = False
        self.is_speaking = False
        self.silence_start_time = None
        self.audio_buffer = []
        self.node.get_logger().info('Service processor reset')

    def get_audio_buffer(self):
        """전체 오디오 버퍼 반환"""
        return np.concatenate(self.audio_buffer) if self.audio_buffer else np.array([])

    def process_result(self, text):
        """STT 결과 처리"""
        if not self.active:
            return
            
        # 텍스트 결과가 있으면 서비스 종료
        if text:
            self.node.get_logger().info(f'Received STT result: {text}')
            self.active = False
            self.reset()
