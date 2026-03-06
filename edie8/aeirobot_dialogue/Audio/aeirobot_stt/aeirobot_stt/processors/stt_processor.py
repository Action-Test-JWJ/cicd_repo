from .stt_engines.stt_factory import STTFactory
from rclpy.node import Node
import numpy as np


class STTProcessor:
    def __init__(self, node: Node):
        """
        STT(Speech-to-Text) 처리 래퍼 클래스
        
        Factory 패턴을 통해 적절한 STT 엔진을 생성하고 관리합니다.

        Args:
            node (Node): ROS2 노드
        """
        self.node = node
        config = node.config
        
        # Factory를 통해 적절한 STT 엔진 생성
        try:
            self.stt_engine = STTFactory.create_stt(node, config)
            self.node.get_logger().info(
                f'STT engine initialized: {self.stt_engine.__class__.__name__}'
            )
        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize STT engine: {str(e)}')
            raise
    
    # def process_audio(self, wav_path: str):
    #     """
    #     WAV 파일을 텍스트로 변환 (하위 호환성 유지)
        
    #     Args:
    #         wav_path (str): WAV 파일 경로
            
    #     Returns:
    #         tuple: (text, language, confidence) 또는 None
    #     """
    #     return self.process_speech_to_text(wav_path)
    
    def process_speech_to_text(self, wav_path: str):
        """
        WAV 파일을 직접 처리하여 STT 수행
        
        Args:
            wav_path (str): 처리할 WAV 파일 경로
            
        Returns:
            tuple: (text, language, confidence) 또는 None
                - text (str): 변환된 텍스트
                - language (str): 감지된 언어 ('ko' 또는 'en')
                - confidence (float): 감지 신뢰도
        """
        try:
            # STT 엔진에 처리 위임
            result = self.stt_engine.transcribe(wav_path)
            
            if result:
                text, language, confidence = result
                
                if text:
                    # 파일 정리 (BaseSTT의 cleanup_files 메서드 사용)
                    self.stt_engine.cleanup_files(wav_path)
                    return text, language, confidence
                else:
                    self.node.get_logger().warn('STT returned empty text')
                    self.stt_engine.cleanup_files(wav_path)
                    return None
            else:
                # 실패 시에도 파일 정리
                self.stt_engine.cleanup_files(wav_path)
                return None
                
        except Exception as e:
            self.node.get_logger().error(f'Error in STT processing: {str(e)}')
            # 에러 시에도 파일 정리 시도
            try:
                self.stt_engine.cleanup_files(wav_path)
            except Exception as cleanup_error:
                self.node.get_logger().warn(f'Failed to cleanup files after error: {str(cleanup_error)}')
            return None

    def process_audio_stream(self, audio_data: np.ndarray, sample_rate: int = None):
        """
        오디오 데이터를 직접 처리하여 STT 수행 (스트림 모드)
        
        파일 저장 없이 메모리 상의 오디오 데이터를 직접 처리합니다.
        
        Args:
            audio_data (np.ndarray): 오디오 데이터 (float32, -1.0 ~ 1.0)
            sample_rate (int): 샘플레이트 (현재 미사용, 엔진에서 16kHz 가정)
            
        Returns:
            tuple: (text, language, confidence) 또는 None
                - text (str): 변환된 텍스트
                - language (str): 감지된 언어 ('ko' 또는 'en')
                - confidence (float): 감지 신뢰도
        """
        try:
            # STT 엔진의 transcribe_stream 메서드 호출
            text = self.stt_engine.transcribe_stream(audio_data, sample_rate)
            
            if text:
                # transcribe_stream은 텍스트만 반환하므로 language와 confidence 추가
                language = self.stt_engine.language if hasattr(self.stt_engine, 'language') else 'ko'
                confidence = 1.0  # 스트림 모드에서는 confidence 정보가 없음
                return text, language, confidence
            else:
                self.node.get_logger().warn('STT stream returned empty text')
                return None
                
        except Exception as e:
            self.node.get_logger().error(f'Error in STT stream processing: {str(e)}')
            import traceback
            traceback.print_exc()
            return None
