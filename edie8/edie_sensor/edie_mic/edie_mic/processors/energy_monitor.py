#!/usr/bin/env python3
"""
오디오 에너지 모니터링 클래스 - dB 계산
ROS2 토픽으로 받은 오디오 데이터의 데시벨 값을 계산
"""

import numpy as np
from rclpy.node import Node
from std_msgs.msg import String

class EnergyMonitor:
    def __init__(self, node: Node, is_monitoring=True):
        """
        오디오 에너지 모니터링 클래스 - dB 기반

        Args:
            node (Node): ROS2 노드
            is_monitoring (bool): 모니터링 모드 여부
        """
        self.node = node
        self.is_monitoring = is_monitoring
        self.latest_db = -100.0  # 최근 dB 값 저장
        
        if is_monitoring:
            self.energy_publisher = node.create_publisher(String, 'audio_energy', 10)

        self.node.get_logger().info('Energy monitor initialized (dB-based)')
    
    
    def preprocess_audio(self, audio_data, encoding='F32LE'):
        """오디오 데이터 전처리

        Args:
            audio_data: 오디오 데이터 (bytes, list, 또는 numpy.ndarray)
            encoding: 오디오 인코딩 형식 (예: 'S16LE', 'F32LE')

        Returns:
            numpy.ndarray: float32 오디오 프레임 (-1.0 ~ 1.0 근처 스케일)
        """
        # array.array, bytes, list 등 다양한 타입 처리 → raw bytes로 통일
        # 실질적으로 sender_node에서 처음부터 bytes로 보내므로 이 부분은 항상 raw_bytes = audio_data 이렇게 됨
        if hasattr(audio_data, 'tobytes'):
            raw_bytes = audio_data.tobytes()
        elif isinstance(audio_data, bytes):
            raw_bytes = audio_data
        else:
            raw_bytes = bytes(audio_data)

        # encoding에 따라 float32 프레임으로 변환
        try:
            if encoding.upper() == 'F32LE':
                # 32비트 float 리틀엔디언
                float_frame = np.frombuffer(raw_bytes, dtype=np.float32)
            else:
                # 기본: 16비트 signed 리틀엔디언 (S16LE)
                int16_frame = np.frombuffer(raw_bytes, dtype=np.int16)
                float_frame = int16_frame.astype(np.float32) / 32768.0

            # DC offset 제거
            if float_frame.size > 0:
                float_frame = float_frame - np.mean(float_frame)

            return float_frame.astype(np.float32)
        except Exception as e:
            self.node.get_logger().error(f'Error in preprocess_audio: {str(e)}')
            return np.array([], dtype=np.float32)
    
    
    
    def calculate_energy(self, audio_frame):
        """오디오 데이터의 데시벨 값을 계산

        Args:
            audio_frame (numpy.ndarray): float32 오디오 프레임 (-1.0 ~ 1.0 근처 스케일)

        Returns:
            float: 데시벨 값 (dB)
        """
        try:
            if not isinstance(audio_frame, np.ndarray) or audio_frame.size == 0:
                return -80.0

            # float32로 보장
            float_frame = audio_frame.astype(np.float32)

            # RMS 계산
            rms = np.sqrt(np.mean(np.square(float_frame)))

            # 데시벨 계산 (기준: 1.0 = 0dB)
            # -inf를 방지하기 위해 작은 값 추가
            db = 20 * np.log10(rms + 1e-10)

            # 최근 dB 값 업데이트
            self.latest_db = db

            return db

        except Exception as e:
            self.node.get_logger().error(f'Error calculating energy: {str(e)}')
            return -100.0  # 매우 낮은 dB값을 반환

    def publish_energy(self, energy_str):
        """dB 레벨 발행"""
        msg = String()
        msg.data = energy_str
        self.energy_publisher.publish(msg)

    def get_latest_db(self):
        """최근 계산된 dB 값 반환"""
        return self.latest_db

    def start_monitoring(self):
        """모니터링 시작"""
        self.is_monitoring = True

    def stop_monitoring(self):
        """모니터링 중지"""
        self.is_monitoring = False
