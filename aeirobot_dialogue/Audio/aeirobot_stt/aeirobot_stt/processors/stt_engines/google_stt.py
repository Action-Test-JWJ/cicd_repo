from .base_stt import BaseSTT
from typing import Optional, Tuple
import logging

# Google Speech Recognition 라이브러리는 lazy import로 처리
try:
    import speech_recognition as sr
    GOOGLE_STT_AVAILABLE = True
except ImportError:
    GOOGLE_STT_AVAILABLE = False
    sr = None


class GoogleSTT(BaseSTT):
    def __init__(self, node, config: dict):
        """
        Google Speech Recognition STT 구현
        
        Args:
            node (Node): ROS2 노드
            config (dict): STT 설정 딕셔너리
        """
        super().__init__(node, config)
        
        # Google STT 사용 가능 여부 확인
        if not GOOGLE_STT_AVAILABLE:
            error_msg = (
                'speech_recognition library not installed. '
                'Run: pip install SpeechRecognition'
            )
            self.logger.error(error_msg)
            raise ImportError(error_msg)
        
        # Google STT 초기화
        self.recognizer = sr.Recognizer()
        self.language = config.get('language', 'ko-KR')
        
        self.logger.info(
            f'Google STT initialized with language: {self.language}'
        )
    
    def transcribe(self, wav_path: str) -> Optional[Tuple[str, str, float]]:
        """
        Google Speech Recognition으로 WAV 파일을 텍스트로 변환
        
        Args:
            wav_path (str): 변환할 WAV 파일 경로
            
        Returns:
            tuple: (text, language, confidence) 또는 None
        """
        try:
            # 오디오 파일 읽기
            with sr.AudioFile(wav_path) as source:
                audio = self.recognizer.record(source)
            
            # Google API로 STT 처리
            text = self.recognizer.recognize_google(
                audio, 
                language=self.language
            )
            
            # 언어 코드 추출 (ko-KR -> ko)
            lang_code = self.language.split('-')[0] if '-' in self.language else self.language
            confidence = 1.0  # Google API는 신뢰도를 제공하지 않으므로 1.0으로 설정
            
            self.logger.info(f'Google STT result: {text}')
            
            # 결과 저장
            self.save_result_to_txt(wav_path, text, lang_code, confidence)
            
            return text, lang_code, confidence
            
        except sr.UnknownValueError:
            self.logger.warn('Google Speech Recognition could not understand audio')
            return None
            
        except sr.RequestError as e:
            self.logger.error(
                f'Could not request results from Google Speech Recognition service: {e}'
            )
            return None
            
        except Exception as e:
            self.logger.error(f'Error in Google STT transcription: {str(e)}')
            return None
