import os
import logging
from typing import Optional, Tuple

# SpeechBrain 라이브러리는 lazy import로 처리
try:
    import soundfile as sf
    from speechbrain.inference.classifiers import EncoderClassifier
    SPEECHBRAIN_AVAILABLE = True
except ImportError:
    SPEECHBRAIN_AVAILABLE = False
    EncoderClassifier = None
    sf = None

MODELS_BASE_DIR = os.path.expanduser("~/.aeirobot_models")
LANG_ID_MODEL_REPO_ID = "speechbrain/lang-id-voxlingua107-ecapa"


class LanguageDetector:
    """
    SpeechBrain VoxLingua107-ECAPA 모델을 사용한 언어 감지 유틸리티
    
    107개 언어를 지원하며, 오디오 파일의 언어를 자동으로 감지합니다.
    """
    
    # 상위 언어 리스트 (우선 처리할 언어들)
    TOP_LANGUAGES = {
        'ko': 'Korean',
        'en': 'English',
        # 필요시 추가 언어
        # 'zh': 'Chinese',
        # 'ja': 'Japanese',
        # 'es': 'Spanish',
    }
    
    def __init__(self, 
                 model_repo_id: str = LANG_ID_MODEL_REPO_ID,
                 base_confidence_threshold: float = 0.40,
                 device: str = "cuda"):
        """
        언어 감지기 초기화
        
        Args:
            model_repo_id (str): SpeechBrain 모델 저장소 ID
            base_confidence_threshold (float): 기본 신뢰도 임계값 (0.0-1.0)
            device (str): 실행 디바이스 ("cuda" 또는 "cpu")
        """
        self.logger = logging.getLogger(__name__)
        
        if not SPEECHBRAIN_AVAILABLE:
            error_msg = (
                'SpeechBrain or soundfile not installed. '
                'Run: pip install speechbrain soundfile'
            )
            self.logger.error(error_msg)
            raise ImportError(error_msg)
        
        self.model_repo_id = model_repo_id
        self.base_confidence_threshold = base_confidence_threshold
        self.device = device
        
        # 모델 로드
        self._load_model()
    
    def _load_model(self):
        """언어 감지 모델 로드"""
        model_path_lang_id = ""
        try:
            lang_id_model_dir_name = self.model_repo_id.split('/')[-1]
            model_path_lang_id = os.path.join(MODELS_BASE_DIR, lang_id_model_dir_name)
            
            self.logger.info(f'Attempting to load Language Detection model from: {model_path_lang_id}')
            
            # 모델 파일 존재 확인
            expected_hyperparams_path = os.path.join(model_path_lang_id, 'hyperparams.yaml')
            if not os.path.exists(expected_hyperparams_path):
                self.logger.error(
                    f"hyperparams.yaml not found at {expected_hyperparams_path}. "
                    f"Ensure models are downloaded to '{MODELS_BASE_DIR}'"
                )
                raise FileNotFoundError(f"hyperparams.yaml not found in {model_path_lang_id}")
            
            # SpeechBrain 모델 로드
            self.language_id_model = EncoderClassifier.from_hparams(
                source=model_path_lang_id,
                run_opts={"device": self.device}
            )
            self.logger.info(f'Language Detection model loaded successfully from {model_path_lang_id}')
            
        except Exception as e:
            self.logger.error(f'Error initializing Language Detection model (tried path: {model_path_lang_id}): {str(e)}')
            raise
    
    def detect(self, audio_path: str) -> Tuple[Optional[str], Optional[float]]:
        """
        오디오 파일의 언어 감지
        
        Args:
            audio_path (str): 음성 파일 경로
            
        Returns:
            tuple: (language_code, confidence) 또는 (None, None)
                - language_code (str): 감지된 언어 코드 ('ko', 'en' 등)
                - confidence (float): 감지 신뢰도 (0.0-1.0)
        """
        try:
            # 오디오 파일 정보 확인
            audio_info = sf.info(audio_path)
            audio_duration = audio_info.duration
            
            # 오디오 길이에 따른 임계값 조정
            adjusted_threshold = self._adjust_threshold(audio_duration)
            
            # 언어 감지 수행
            audio_lang_data = self.language_id_model.load_audio(audio_path)
            prediction = self.language_id_model.classify_batch(audio_lang_data)
            
            # 언어 코드 추출 (콜론 앞부분만)
            lang_str = prediction[3][0] if isinstance(prediction[3], list) and prediction[3] else 'unknown'
            lang_code = lang_str.split(':')[0].strip() if ':' in lang_str else lang_str
            
            # 신뢰도 값 추출
            confidence_value = float(prediction[1].exp().item()) if hasattr(prediction[1].exp(), 'item') else 0.0
            
            self.logger.debug(
                f'Detected language: {lang_code} with confidence: {confidence_value:.4f} '
                f'(threshold: {adjusted_threshold:.2f}, duration: {audio_duration:.2f}s)'
            )
            
            # 언어 규칙 적용
            return self._apply_language_rules(lang_code, confidence_value, adjusted_threshold)
            
        except Exception as e:
            self.logger.error(f'Error in language detection: {str(e)}')
            return None, None
    
    def _adjust_threshold(self, audio_duration: float) -> float:
        """
        오디오 길이에 따른 임계값 동적 조정
        
        짧은 음성일수록 언어 감지가 어려우므로 임계값을 높입니다.
        
        Args:
            audio_duration (float): 오디오 길이 (초)
            
        Returns:
            float: 조정된 임계값
        """
        adjusted_threshold = self.base_confidence_threshold
        
        if audio_duration < 1.0:
            # 1초 미만: 임계값 50% 상향
            adjusted_threshold = self.base_confidence_threshold * 1.5
            self.logger.debug(f'Short audio ({audio_duration:.2f}s): Increasing threshold to {adjusted_threshold:.2f}')
        elif audio_duration < 2.0:
            # 2초 미만: 임계값 25% 상향
            adjusted_threshold = self.base_confidence_threshold * 1.25
            self.logger.debug(f'Medium audio ({audio_duration:.2f}s): Increasing threshold to {adjusted_threshold:.2f}')
        
        return adjusted_threshold
    
    def _apply_language_rules(self, 
                             lang_code: str, 
                             confidence_value: float, 
                             threshold: float) -> Tuple[Optional[str], float]:
        """
        언어 감지 결과에 규칙 적용
        
        상위 언어와 비상위 언어를 구분하여 다른 임계값을 적용합니다.
        
        Args:
            lang_code (str): 감지된 언어 코드
            confidence_value (float): 언어 감지 신뢰도
            threshold (float): 현재 상황에 맞는 임계값
            
        Returns:
            tuple: (final_lang_code, confidence_value)
                - final_lang_code (str or None): 최종 결정된 언어 코드 또는 None
                - confidence_value (float): 언어 감지 신뢰도
        """
        # 상위 언어 여부 확인
        if lang_code in self.TOP_LANGUAGES:
            # 상위 언어인 경우 - 원래 임계값 적용
            if confidence_value >= threshold:
                self.logger.debug(f'Using detected language: {lang_code} ({self.TOP_LANGUAGES[lang_code]})')
                return lang_code, confidence_value
            else:
                # 상위 언어지만 신뢰도가 낮은 경우
                self.logger.debug(f'Confidence too low for top language ({confidence_value:.4f} < {threshold:.2f})')
                return None, confidence_value
        else:
            # 상위 언어가 아닌 경우 - 더 낮은 임계값 적용
            lower_threshold = threshold * 0.7  # 임계값 30% 낮춤
            if confidence_value >= lower_threshold:
                self.logger.debug(f'Non-top language {lang_code} with sufficient confidence: defaulting to Korean')
                return 'ko', confidence_value  # 한국어로 기본 설정
            else:
                # 비상위 언어이면서 낮춘 임계값도 못 넘는 경우
                self.logger.debug(f'Confidence too low for non-top language ({confidence_value:.4f} < {lower_threshold:.2f})')
                return None, confidence_value
