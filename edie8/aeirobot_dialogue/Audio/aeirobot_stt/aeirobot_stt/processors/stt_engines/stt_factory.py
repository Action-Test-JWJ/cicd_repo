from .localhost_stt import LocalhostSTT
from .google_stt import GoogleSTT
from .whisper_stt import WhisperSTT
from .sensevoice_stt import SenseVoiceSTT


class STTFactory:
    @staticmethod
    def create_stt(node, config: dict):
        """
        STT 인스턴스 생성

        config에서 'stt_mode' 필드를 읽어 STT 타입에 맞는 인스턴스를 생성합니다.
        config에 없는 인자는 각 STT 클래스의 기본값이 사용됩니다.

        Args:
            node (Node): ROS2 노드
            config (dict): STT 설정 딕셔너리 (node.config에서 전달됨)

        Returns:
            BaseSTT: STT 인스턴스

        Raises:
            ValueError: 알 수 없는 STT 모드인 경우
        """
        stt_mode = config.get('stt_mode', 'localhost')
        
        if stt_mode.lower() == 'localhost':
            return LocalhostSTT(node, config)
        elif stt_mode.lower() == 'google':
            return GoogleSTT(node, config)
        elif stt_mode.lower() == 'whisper':
            return WhisperSTT(node, config)
        elif stt_mode.lower() == 'sensevoice':
            return SenseVoiceSTT(node, config)
        else:
            raise ValueError(
                f'Unknown STT mode: {stt_mode}. '
                f'Available modes: google, localhost, whisper, sensevoice'
            )
