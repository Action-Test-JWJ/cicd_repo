# AeiROBOT STT

## STT (Speech To Text): 음성 발화 텍스트 변환 

[Back to Top](../README.md)

## 기능

발화 구역으로 저장된 발화 음성 파일에서 텍스트로 변환 

## 사용법 


### 1. 발화 음성 파일에서 텍스트로 변환

```
ros2 run aeirobot_stt stt_main
```


## Topic

`aeirobot_asr/config/stt_config.yaml` 에서 관리

| Topic Name                 | Type   | Pub/Sub |
| -------------------------- | ------ | ------- |
| /aeirobot/asr/status       | Bool   | subscribe |
| /aeirobot/asr/file         | String | subscribe |
| /aeirobot/stt/result       | String   | publish |

`/aeirobot/asr/status` : 
- 음성 인식의 상태를 나타냅니다.
- 발화가 인식되는 순간 True, 발화가 끝나는 순간 False
- False가 되는 순간 file을 불러와서 stt를 진행합니다.

`/aeirobot/asr/file` : 
- 저장된 파일 위치를 구독합니다.
- stt가 진행될때 해당 위치(이름)의 파일을 사용합니다.

`/aeirobot/stt/result` : 
- stt가 처리한 텍스트를 발행합니다.


## 구조 

```
aeirobot_stt/
├── aeirobot_stt/
│   ├── __init__.py
│   ├── stt_main.py             # wav 파일에서 텍스트 추출 후 발행
├── processors/
│   ├── service_processor.py    # service 관리자 
│   ├── stt_processor.py        # stt 실행자
│   └── stt_engines             # STT 모델 관리
│       ├── base_stt.py
│       ├── google_stt.py
│       ├── localhost_stt.py
│       ├── whsiper_stt.py
│       ├── sensevoice_stt.py
│       └── stt_factory.py
├── config/
│   └── stt_config.yaml     # 오디오 설정 파일
├── resource/
│   └── aeirobot_stt
├── package.xml
├── setup.py
├── setup.cfg
├── requirements.txt
└── README.md
```


## Deprecated

### Setup & Installation


### Dependency


## ToDo
- [] service 관리
- [] action 관리