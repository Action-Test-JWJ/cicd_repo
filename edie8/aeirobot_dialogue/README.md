# AeiRobot Dialogue

## Aeirobot Audio

[설명서](Audio/README.md)


## Aeirobot LLM

[설명서](LLM/README.md)


## Aeirobot Vision

[설명서](Vision/README.md)



## Update Log

### 0.1.0 (2025-12)
- [ASR] Processor들과 main 코드 분리 및 구조 변경
- [ASR] ASR에서 VAD 처리와 STT 처리 부분 분리
- [STT] STT 부분 추가 : VAD에서 처리된 발화 파일 불러와서 실행


### 0.0.2 (2024-12)
- [ASR] config launch file for several type of robots
- [ASR] add service call
- [Interface] config srv file for STT, LLM, TTS

### 0.0.1 (2024-12)
- Init Repository
- Add heroehs_asr package
- Add aimy_llm package
- Add aimy_tts package
- Add aimy_gesture package
- Add heroehs_audio_player package

## TODO

- [ ] STT : SenseVoice 확인 및 rockchip 기반 사용 확인
- [ ] STT : Docker로 관리
- [ ] Vision: Object Detection, Object Tracking, Segmentation 등 관리