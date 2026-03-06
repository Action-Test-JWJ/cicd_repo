from .base_stt import BaseSTT
from typing import Optional, Tuple
import logging

# API 클라이언트는 lazy import로 처리
try:
    from aeirobot_stt.api_client.stt_api import call_stt_api
    API_CLIENT_AVAILABLE = True
except ImportError:
    API_CLIENT_AVAILABLE = False
    call_stt_api = None


class LocalhostSTT(BaseSTT):
    def __init__(self, node, config: dict):
        """
        Localhost API (Whisper) STT 구현
        
        Args:
            node (Node): ROS2 노드
            config (dict): STT 설정 딕셔너리
        """
        super().__init__(node, config)
        
        # API 클라이언트 사용 가능 여부 확인
        if not API_CLIENT_AVAILABLE:
            error_msg = (
                'API client module not found. '
                'Ensure aeirobot_stt.api_client.stt_api is available.'
            )
            self.logger.error(error_msg)
            raise ImportError(error_msg)
        
        # API 설정 로드
        self.api_config = config.get('api', {})
        
        # 기본값 설정
        if 'host' not in self.api_config:
            self.api_config['host'] = 'localhost'
        if 'port' not in self.api_config:
            self.api_config['port'] = 8000
        
        self.logger.info(
            f'Localhost API STT initialized '
            f'(host: {self.api_config["host"]}, port: {self.api_config["port"]})'
        )
    
    def transcribe(self, wav_path: str) -> Optional[Tuple[str, str, float]]:
        """
        Localhost API를 호출하여 WAV 파일을 텍스트로 변환
        
        Args:
            wav_path (str): 변환할 WAV 파일 경로
            
        Returns:
            tuple: (text, language, confidence) 또는 None
        """
        try:
            # Localhost API 호출
            text, lang_code, confidence = call_stt_api(
                wav_path,
                api_conf=self.api_config,
                node=self.node
            )
            
            if text:
                self.logger.info(
                    f'Localhost API STT result: {text} '
                    f'(lang: {lang_code}, conf: {confidence:.2f})'
                )
                
                # 결과 저장
                self.save_result_to_txt(wav_path, text, lang_code, confidence)
                
                return text, lang_code, confidence
            else:
                self.logger.warn('Localhost API returned empty text')
                return None
                
        except Exception as e:
            self.logger.error(f'Error in Localhost API STT transcription: {str(e)}')
            return None
