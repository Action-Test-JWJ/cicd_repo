import os
import numpy as np
from scipy import signal
from .base_stt import BaseSTT

from funasr import AutoModel
from funasr.utils.postprocess_utils import rich_transcription_postprocess
from huggingface_hub import snapshot_download

LOCAL_DIR = os.path.join(os.path.expanduser("~"), ".aeirobot_models", "SenseVoice-Small-ko")


class SenseVoiceSTT(BaseSTT):
    def __init__(self, node, config: dict):
        """
        SenseVoice STT 구현
        
        Args:
            node: ROS2 노드
            config: 설정 딕셔너리
        """
        super().__init__(node, config)
        
        # SenseVoice 전용 설정
        self.model_dir = config.get('model_dir', 'AeiROBOT/SenseVoice-Small-ko') # FunAudioLLM/SenseVoiceSmall   # AeiROBOT/SenseVoice-Small-ko
        self.device = config.get('device', 'cuda:0')
        self.language = config.get('language', 'ko')
        
        self.logger.info(f"Loading SenseVoice model on {self.device}...")
        
        # Hugging Face Private Repo 접근을 위해 명시적 다운로드 시도
        # AutoModel이 비공개 저장소 인증을 제대로 처리하지 못할 경우를 대비해 huggingface_hub로 먼저 다운로드
        if not os.path.exists(self.model_dir):
            try:
                self.logger.info(f"Downloading model from Hugging Face: {self.model_dir}")
                # 환경 변수에서 토큰을 명시적으로 가져와 전달 (우선순위 확보)
                # token = os.environ.get("HUGGINGFACE_HUB_TOKEN")
                # self.model_dir = snapshot_download(repo_id=self.model_dir, token=token)
                self.model_dir = snapshot_download(
                    repo_id=self.model_dir,
                    repo_type="model",
                    local_dir=LOCAL_DIR,
                    local_dir_use_symlinks=False,
                    token=os.environ.get("HUGGINGFACE_HUB_TOKEN"),  # private 이므로 필요
                )

                self.logger.info(f"Model downloaded to: {self.model_dir}")
            except Exception as e:
                self.logger.error(f"Failed to download model via snapshot_download: {e}")
        
        self.model = AutoModel(
            model=self.model_dir,
            trust_remote_code=True,
            device=self.device,
            hub="hf",  # HuggingFace에서 다운로드
        )
        
        # self.model = AutoModel(
        #     model=model_dir,
        #     vad_model="fsmn-vad",
        #     vad_kwargs={"max_single_segment_time": 30000},
        #     device="cuda:0",
        #     hub="hf",
        # )
        
        self.logger.info("SenseVoice model loaded!")
    
    
    def transcribe(self, file_path: str):
        """
        오디오 파일을 텍스트로 변환
        
        Args:
            file_path: 오디오 파일 경로
        
        Returns:
            tuple: (text, language, confidence)
        """
        res = self.model.generate(
            input=file_path,
            cache={},
            language=self.language,
            use_itn=True,
            batch_size=1,
        )
        
        if res and len(res) > 0:
            text = rich_transcription_postprocess(res[0]["text"])
            return text, self.language, 1.0  # (text, language, confidence)
        return None
    
    
    def transcribe_stream(self, audio_data: np.ndarray, sample_rate: int = 48000) -> str:
        """
        오디오 데이터를 텍스트로 변환
        
        Args:
            audio_data: numpy array (float32 권장)
            sample_rate: 입력 오디오 샘플레이트 (기본값: 48000Hz)
        
        Returns:
            인식된 텍스트
        """
        # float32로 변환
        if audio_data.dtype != np.float32:
            audio_data = audio_data.astype(np.float32)
        
        # int16 범위였다면 정규화 (-1 ~ 1)
        if np.max(np.abs(audio_data)) > 1.0:
            audio_data = audio_data / 32768.0
        
        # 샘플레이트가 16kHz가 아니면 리샘플링 (SenseVoice는 16kHz 기대)
        target_sample_rate = 16000
        if sample_rate and sample_rate != target_sample_rate:
            num_samples = int(len(audio_data) * target_sample_rate / sample_rate)
            audio_data = signal.resample(audio_data, num_samples)
            self.logger.debug(f'Resampled audio from {sample_rate}Hz to {target_sample_rate}Hz: {len(audio_data)} samples')
        
        res = self.model.generate(
            input=audio_data,
            cache={},
            language=self.language,
            use_itn=True,
            # batch_size=1,
        )
        
        if res and len(res) > 0:
            text = rich_transcription_postprocess(res[0]["text"])
            return text
        return ""
    
    def get_info(self) -> dict:
        """STT 설정 정보 반환"""
        info = super().get_info()
        info.update({
            'model_dir': self.model_dir,
            'device': self.device,
        })
        return info
