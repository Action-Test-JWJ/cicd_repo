# edie_mic

ROS2 오디오 스트리밍 패키지. GStreamer와 PulseAudio/PipeWire를 사용하여 마이크 입력을 ROS2 토픽으로 스트리밍하고, 토픽 데이터를 스피커로 출력합니다.

## 구조

```
edie_mic/
├── edie_mic/
│   ├── __init__.py
│   └── audio_node.py    # ROS2 토픽 → dB 계산 및 시각화와 스피커
├── config/
│   └── audio_config.yaml   # 오디오 설정 파일
├── resource/
│   └── edie_mic
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 의존성

### 시스템 패키지

```bash
sudo apt update
sudo apt install -y \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-pulseaudio \
    gstreamer1.0-alsa \
    python3-gi \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0
```

### ROS2 패키지

- `rclpy`
- `audio_msgs` (ros-gst-bridge에서 제공)

```bash
# ros-gst-bridge 설치 (audio_msgs 포함)
cd ~/ros2_ws/src
git clone https://github.com/BrettRD/ros-gst-bridge.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select audio_msgs
```

## 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select edie_mic
source install/setup.bash
```

## 설정

### config/audio_config.yaml

```yaml
aeirobot_asr:
  ros__parameters:
    pub:
      pub_audio: '/aeirobot/edie/audio'

  audio__parameters:
    sample_rate: 48000
    default_device: DeepFilterMic  # PulseAudio 소스 이름
    channels: 1
    output_device: ''              # 비워두면 기본 출력 장치 사용
```

### 오디오 장치 확인

```bash
# PulseAudio 입력 장치 목록
pactl list sources short

# PulseAudio 출력 장치 목록
pactl list sinks short

# 기본 소스/싱크 확인
pactl info | grep -E "기본 소스|기본 싱크|Default Source|Default Sink"
```

## 사용법

### Sender Node (마이크 → 토픽)

```bash
# 기본 설정으로 실행
ros2 run edie_mic audio_node

# 파라미터 지정
ros2 run edie_mic audio_node --ros-args \
    -p device:=DeepFilterMic \
    -p sample_rate:=48000 \
    -p channels:=1 \
    -p topic:=/aeirobot/edie/audio


```


## ROS2 토픽

### /aeirobot/edie/audio

| 항목 | 값 |
|------|-----|
| 타입 | `audio_msgs/msg/Audio` |
| Publisher | `audio_node` |

### 메시지 구조 (audio_msgs/msg/Audio)

```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id
uint32 sample_rate      # 샘플레이트 (Hz)
uint8 channels          # 채널 수
string encoding         # 인코딩 (S16LE, F32LE 등)
uint8[] data            # PCM 오디오 데이터
```

### 토픽 모니터링

```bash
# 토픽 목록 확인
ros2 topic list

# 토픽 주파수 확인
ros2 topic hz /aeirobot/edie/audio

# 토픽 정보 확인
ros2 topic info /aeirobot/edie/audio

# 메시지 확인 (데이터 배열 제외)
ros2 topic echo /aeirobot/edie/audio --no-arr
```

약 100Hz


## 파라미터

### sender_node

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `config_file` | string | `../config/audio_config.yaml` | 설정 파일 경로 |
| `device` | string | `DeepFilterMic` | PulseAudio 입력 장치 |
| `sample_rate` | int | `48000` | 샘플레이트 (Hz) |
| `channels` | int | `1` | 채널 수 |
| `topic` | string | `/aeirobot/edie/audio` | 퍼블리시 토픽 |

### receiver_node

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `config_file` | string | `../config/audio_config.yaml` | 설정 파일 경로 |
| `device` | string | `""` | PulseAudio 출력 장치 (비어있으면 기본 장치) |
| `topic` | string | `/aeirobot/edie/audio` | 구독 토픽 |

## 장치 Fallback

### sender_node

```
1. config의 default_device (예: DeepFilterMic) 시도
   ↓ 실패
2. PulseAudio 기본 소스 사용
   ↓ 실패
3. 에러 로그 출력
```


## DeepFilterNet 연동

[DeepFilterNet](https://github.com/Rikorose/DeepFilterNet) 노이즈 캔슬링을 사용하려면:

1. DeepFilterNet LADSPA 플러그인 설치
2. PipeWire filter-chain 설정 (`~/.config/pipewire/filter-chain.conf`)
3. `default_device: DeepFilterMic` 설정

DeepFilterMic이 없는 환경에서는 자동으로 기본 마이크를 사용합니다.

## 트러블슈팅

### "No audio device available" 에러

```bash
# GStreamer PulseAudio 플러그인 확인
gst-inspect-1.0 pulsesrc
gst-inspect-1.0 pulsesink

# 없으면 설치
sudo apt install gstreamer1.0-pulseaudio

# 직접 테스트
gst-launch-1.0 pulsesrc ! fakesink
gst-launch-1.0 audiotestsrc ! pulsesink
```

### 특정 장치 테스트

```bash
# 입력 장치 테스트
gst-launch-1.0 pulsesrc device="DeepFilterMic" ! fakesink

# 출력 장치 테스트
gst-launch-1.0 audiotestsrc ! pulsesink device="alsa_output.pci-0000_00_1f.3.analog-stereo"
```

### PulseAudio 재시작

```bash
systemctl --user restart pipewire pipewire-pulse
```

--------


# 정리

### 1. DeepFilterNet - 시스템/플러그인 단위
```
┌─────────────────────────────────────────────────────────┐
│                   PipeWire (시스템)                      │
│  ┌─────────────────────────────────────────────────┐   │
│  │  filter-chain.conf (LADSPA 플러그인)             │   │
│  │  └─ DeepFilterNet 노이즈 캔슬링                  │   │
│  │     └─ 가상 소스: "DeepFilterMic" 으로 노출      │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

- 설정 위치: ~/.config/pipewire/filter-chain.conf
- 플러그인: ~/.ladspa/libdeep_filter_ladspa-*.so


### 2. GStreamer - 시스템/플러그인 단위

```
┌─────────────────────────────────────────────────────────┐
│                   GStreamer (시스템)                     │
│  ┌─────────────────────────────────────────────────┐   │
│  │  플러그인들 (/usr/lib/.../gstreamer-1.0/)        │   │
│  │  ├─ pulsesrc (PulseAudio 입력)                  │   │
│  │  ├─ pulsesink (PulseAudio 출력)                 │   │
│  │  ├─ audioconvert (포맷 변환)                    │   │
│  │  └─ audioresample (리샘플링)                    │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```
- 설치 위치: sudo apt install gstreamer1.0-*
- 어플리케이션은: 파이프라인 문자열만 전달

### 전체 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                        시스템 레벨                               │
├─────────────────────────────────────────────────────────────────┤
│  마이크 (hw:2,0)                                                │
│       ↓                                                         │
│  PipeWire + DeepFilterNet (LADSPA 플러그인)                     │
│       ↓                                                         │
│  가상 소스: "DeepFilterMic"                                     │
│       ↓                                                         │
│  GStreamer (pulsesrc → audioconvert → audioresample)            │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            │                            
                            │
┌───────────────────────────┴─────────────────────────────────────┐
│                      어플리케이션 레벨                           │
├─────────────────────────────────────────────────────────────────┤
│  sender_node.py: 토픽 publish만                                 │
│  receiver_node.py: 토픽 subscribe만                             │
│  (무거운 오디오 처리는 전혀 없음)                                 │
└─────────────────────────────────────────────────────────────────┘
                            │
                       /aeirobot/edie/audio (ROS2 토픽)
                            │
```