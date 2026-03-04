from abc import ABC, abstractmethod
import os
import logging
from typing import Optional, Tuple
import numpy as np

class BaseSTT(ABC):
    def __init__(self, node, config: dict):
        """
        STT(Speech To Text) 기본 클래스

        Args:
            node (Node): ROS2 노드
            config (dict): STT 설정 딕셔너리
        """
        self.node = node
        self.config = config
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # 공통 설정 파라미터
        self.sample_rate = config.get('sample_rate', 48000)
        self.cleanup_wav = config.get('cleanup_wav', True)
        
        # 로그 디렉토리 설정
        log_dir = config.get('log_dir', '0_models/stt_logs')
        self.temp_dir = os.path.join(os.path.expanduser('~'), log_dir)
        os.makedirs(self.temp_dir, exist_ok=True)
        
        self.logger.info(f'{self.__class__.__name__} initialized')

    @abstractmethod
    def transcribe(self, wav_path: str) -> Optional[Tuple[str, str, float]]:
        """
        WAV 파일을 텍스트로 변환 (각 STT 엔진이 구현해야 함)

        Args:
            wav_path (str): 변환할 WAV 파일 경로

        Returns:
            tuple: (text, language, confidence) 또는 None
                - text (str): 인식된 텍스트
                - language (str): 감지된 언어 코드 ('ko', 'en' 등)
                - confidence (float): 인식 신뢰도 (0.0 ~ 1.0)
        """
        pass


    def transcribe_stream(self, audio_data: np.ndarray, sample_rate: int = None) -> str:
        """
        오디오 데이터를 텍스트로 변환
        
        Args:
            audio_data (np.ndarray): 입력 오디오 데이터
            sample_rate (int): 오디오 샘플레이트 (None이면 self.sample_rate 사용)
        
        Returns:
            str: 인식된 텍스트
        """
        pass


    def cleanup_files(self, wav_path: str, cleanup_txt: bool = True):
        """
        WAV 파일 및 관련 파일 정리

        Args:
            wav_path (str): 삭제할 WAV 파일 경로
            cleanup_txt (bool): TXT 파일도 함께 삭제할지 여부
        """
        if not self.cleanup_wav:
            return
            
        try:
            # WAV 파일 삭제
            if os.path.exists(wav_path):
                os.remove(wav_path)
                self.logger.debug(f'Removed WAV file: {wav_path}')
            
            # TXT 파일 삭제
            if cleanup_txt:
                txt_path = os.path.splitext(wav_path)[0] + '.txt'
                if os.path.exists(txt_path):
                    os.remove(txt_path)
                    self.logger.debug(f'Removed TXT file: {txt_path}')
                    
        except Exception as e:
            self.logger.warn(f'Failed to cleanup files: {str(e)}')

    def save_result_to_txt(self, wav_path: str, text: str, language: str, confidence: float):
        """
        STT 결과를 텍스트 파일로 저장

        Args:
            wav_path (str): 원본 WAV 파일 경로
            text (str): 인식된 텍스트
            language (str): 감지된 언어
            confidence (float): 인식 신뢰도
        """
        txt_path = os.path.splitext(wav_path)[0] + '.txt'
        try:
            with open(txt_path, 'w', encoding='utf-8') as f:
                f.write(f'Language: {language}\n')
                f.write(f'Confidence: {confidence:.2f}\n')
                f.write(f'Text: {text}\n')
            self.logger.debug(f'STT result saved to: {txt_path}')
        except Exception as e:
            self.logger.warn(f'Failed to save STT result to txt: {str(e)}')

    def get_stt_info(self) -> dict:
        """
        현재 STT 설정 정보 반환

        Returns:
            dict: STT 설정 정보
        """
        return {
            'engine': self.__class__.__name__,
            'sample_rate': self.sample_rate,
            'cleanup_wav': self.cleanup_wav,
            'temp_dir': self.temp_dir
        }
