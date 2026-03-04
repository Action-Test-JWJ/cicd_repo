# AeiROBOT ASR

## ASR(Automatic Speech Recognition): 자동 음성 인식 

[Back to Top](../README.md)

## 기능

사람의 발화를 인식하고 사람의 발화 세그멘테이션 후 발화 음성 파일 저장 

- 스트림 모드: wav 파일 저장 안하고 발화 청크를 직접 ros2로 발행 




## 사용법 


### 1. 발화 인식 및 발화 분할 음성 파일 저장 

```
ros2 run aeirobot_asr asr_main
```


## Topic

`aeirobot_asr/config/audio_config.yaml` 에서 관리

| Topic Name                 | Type   | Pub/Sub |
| -------------------------- | ------ | ------- |
| /aeirobot/asr/status       | Bool   | publish |
| /aeirobot/asr/file         | String | publish |


`/aeirobot/asr/status` : 
- 음성 인식의 상태를 나타냅니다.
- 발화가 인식되는 순간 True, 발화가 끝나는 순간 False를 발행합니다.

`/aeirobot/asr/file` : 
- 발화 세그멘테이션이 끝나는 순간 발화 청크를 파일로 저장하고 저장된 파일 위치를 발행합니다. 

## 구조 

```
aeirobot_asr/
├── aeirobot_asr/
│   ├── __init__.py
│   ├── asr_main.py             # ROS2 Audio 토픽 -> VAD -> 발화 인식 및 파일 저장
│   └── record.py               # ROS2 Audio 토픽 -> 파일 저장
├── processors/
│   ├── audio_manager.py        # 다른 모듈에서 사용할 수 있게 데이터 변환 및 버퍼 관리와 저장 
│   ├── energy_monitor.py       # dB 계산 
│   ├── speech_processor.py     # VAD 및 audio segmentation
│   └── vad_engines             # VAD 모델 관리
│       ├── __init__.py
│       ├── base_vad.py
│       ├── silero_vad.py
│       ├── webrtc_vad.py
│       └── vad_factory.py
├── config/
│   └── audio_config.yaml     # 오디오 설정 파일
├── launch/
│   ├── aimy.launch.py
│   ├── alice.launch.py
│   ├── edie.launch.py
│   └── orin.launch.py
├── resource/
│   └── aeirobot_asr
├── package.xml
├── setup.py
├── setup.cfg
├── requirements.txt
└── README.md
```


## Deprecated

### Setup & Installation

PipeWire for MV88+ : https://gist.github.com/the-spyke/2de98b22ff4f978ebf0650c90e82027e?permalink_comment_id=3976215

```

pip install torch==2.5.0 torchvision==0.20.0 torchaudio==2.5.0 --index-url https://download.pytorch.org/whl/cpu

pip install -q huggingface_hub

pip install -q onnxruntime

pip install -q empy

pip install SoundCard

pip install soundfile

```

### Dependency

VAD Model : https://pytorch.org/hub/snakers4_silero-vad_vad/

STT Model : https://huggingface.co/openai/whisper-large-v3
