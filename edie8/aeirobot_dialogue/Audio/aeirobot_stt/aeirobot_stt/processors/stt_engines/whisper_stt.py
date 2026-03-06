from .base_stt import BaseSTT
from typing import Optional, Tuple
import os
import time

# Faster Whisper 라이브러리는 lazy import로 처리
try:
    from faster_whisper import WhisperModel
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    WhisperModel = None

MODELS_BASE_DIR = os.path.expanduser("~/.aeirobot_models")
STT_MODEL_REPO_ID = "faster-whisper-small-ct2" #"deepdml/faster-whisper-large-v3-turbo-ct2"


class WhisperSTT(BaseSTT):
    def __init__(self, node, config: dict):
        """
        Faster Whisper 모델을 직접 로드하여 사용하는 STT
        
        Args:
            node (Node): ROS2 노드
            config (dict): STT 설정 딕셔너리
        """
        super().__init__(node, config)
        
        # Whisper 사용 가능 여부 확인
        if not WHISPER_AVAILABLE:
            error_msg = (
                'faster_whisper library not installed. '
                'Run: pip install faster-whisper'
            )
            self.logger.error(error_msg)
            raise ImportError(error_msg)
        
        # 모델 설정
        self.model_repo_id = config.get('model_repo_id', STT_MODEL_REPO_ID)
        self.device = config.get('device', 'cuda')
        
        # device에 따라 자동으로 compute_type 설정
        if 'cpu' in self.device.lower():
            default_compute_type = 'int8'  # CPU는 int8이 가장 효율적
        else:
            default_compute_type = 'float16'  # GPU는 float16
        self.compute_type = config.get('compute_type', default_compute_type)
        
        self.beam_size = config.get('beam_size', 5)
        self.base_confidence_threshold = config.get('confidence_threshold', 0.40)
        
        # 모델 로드
        self._load_model()
    
    def _load_model(self):
        """Whisper STT 모델 로드"""
        try:
            stt_model_dir_name = self.model_repo_id.split('/')[-1]
            model_path_stt = os.path.join(MODELS_BASE_DIR, stt_model_dir_name)
            
            self.logger.info(f'Attempting to load Whisper STT model from: {model_path_stt}')
            
            # 모델 존재 확인
            expected_config_path = os.path.join(model_path_stt, 'config.json')
            if not os.path.exists(expected_config_path):
                self.logger.error(
                    f"config.json not found at {expected_config_path}. "
                    f"Ensure models are downloaded to '{MODELS_BASE_DIR}'"
                )
                raise FileNotFoundError(f"config.json not found in {model_path_stt}")
            
            # Whisper 모델 로드
            self.stt_model = WhisperModel(
                model_path_stt,
                device=self.device,
                compute_type=self.compute_type
            )
            self.logger.info(f'Whisper model loaded successfully from {model_path_stt}')
            
        except Exception as e:
            self.logger.error(f'Error initializing Whisper model: {str(e)}')
            raise
    
    def transcribe(self, wav_path: str) -> Optional[Tuple[str, str, float]]:
        """
        Whisper로 WAV 파일을 텍스트로 변환
        
        Args:
            wav_path (str): 변환할 WAV 파일 경로
            
        Returns:
            tuple: (text, language, confidence) 또는 None
        """
        try:
            # 첫 번째 STT 처리 (언어 자동 감지)
            start_time = time.time()
            segments, info = self.stt_model.transcribe(
                wav_path,
                beam_size=self.beam_size,
                language=None  # 자동 감지
            )
            text = " ".join(segment.text for segment in segments)
            processing_time = time.time() - start_time
            
            self.logger.debug(f'Whisper STT processing time: {processing_time:.2f} seconds')
            
            # Whisper의 언어 감지 결과 확인
            if info and hasattr(info, 'language') and hasattr(info, 'language_probability'):
                lang_class = info.language
                confidence = info.language_probability
                
                self.logger.debug(
                    f'Whisper language detection: {lang_class} with confidence: {confidence:.4f}'
                )
                
                # 한국어나 영어가 아니거나 낮은 확률일 경우 다시 실행
                if (lang_class not in ['ko', 'en']) or (confidence < self.base_confidence_threshold):
                    self.logger.info(
                        f'Low confidence or non-Korean/English detection: {lang_class} '
                        f'({confidence:.4f}). Retrying with Korean...'
                    )
                    return self._retry_with_korean(wav_path)
                
                # 결과 저장
                self.save_result_to_txt(wav_path, text, lang_class, confidence)
                return text, lang_class, confidence
            
            # 언어 정보가 없으면 한국어로 재시도
            self.logger.info('No language info detected, retrying with Korean...')
            return self._retry_with_korean(wav_path)
            
        except Exception as e:
            self.logger.error(f'Error in Whisper STT transcription: {str(e)}')
            return None
    
    def _retry_with_korean(self, wav_path: str) -> Optional[Tuple[str, str, float]]:
        """한국어로 강제 설정하여 재시도"""
        try:
            self.logger.debug('Forcing Korean language mode and retrying transcription')
            retry_start_time = time.time()
            
            retry_segments, retry_info = self.stt_model.transcribe(
                wav_path,
                beam_size=self.beam_size,
                language='ko'  # 한국어로 강제 설정
            )
            retry_text = " ".join(segment.text for segment in retry_segments)
            retry_time = time.time() - retry_start_time
            
            self.logger.debug(f'Retry with Korean mode - processing time: {retry_time:.2f} seconds')
            
            # 결과 저장
            self.save_result_to_txt(wav_path, retry_text, 'ko', 1.0)
            return retry_text, 'ko', 1.0  # 강제 설정했으니 확신도 1.0
            
        except Exception as e:
            self.logger.error(f'Error in Korean retry: {str(e)}')
            return None
