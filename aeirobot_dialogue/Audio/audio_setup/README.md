# DeepFilterNet 설정 방법

### Host PC의 경우 
- **아키텍처**: x86_64

### Orin의 경우 
- **아키텍처**: aarch64

아키텍처 확인하는 법
```
uname -m
```

## 단계
1. PipeWire 설치 확인
2. DeepFilterNet LADSPA 플러그인 다운로드
3. PipeWire-Pulse 설치
4. WirePlumber 설치
5. 필터 체인 설정
6. systemd 서비스 생성
7. 서비스 활성화
8. 확인 및 테스트
9. 기본 마이크 설정
10. 참고: 대안 방법 (제공한 파일 사용)

## 각 단계별 간단 설명 
1. PipeWire 설치 확인
    - PipeWire가 설치되어 있고 실행 중인지 확인
2. DeepFilterNet LADSPA 플러그인 다운로드
    - DeepFilter LADSPA 플러그인을 다운로드
3. PipeWire-Pulse 설치
    - PulseAudio를 PipeWire-Pulse로 교체
4. WirePlumber 설치
    - WirePlumber는 PipeWire의 세션 매니저로, ALSA 오디오 장치를 자동으로 관리
5. 필터 체인 설정
    - PipeWire 필터 체인 설정 파일 생성
6. systemd 서비스 생성
    - 부팅 시 자동으로 DeepFilter가 시작되도록 systemd 서비스 생성
7. 서비스 활성화
    - pipewire, wireplumber, pipewire-pulse, pipewire-filter-chain 서비스 활성화
8. 확인 및 테스트
    - 서비스, 장치 확인 및 녹음 테스트
9. 기본 마이크 설정
    - DeepFilterMic을 시스템 기본 마이크로 설정


---

# 1단계: PipeWire 확인

PipeWire가 설치되어 있고 실행 중인지 확인합니다.

```
# PipeWire 버전 확인
pipewire --version

# PipeWire 서비스 상태 확인
systemctl --user status pipewire
```

# 2단계: DeepFilterNet LADSPA 플러그인 다운로드

```
# .ladspa 디렉토리 생성
mkdir -p ~/.ladspa

# 디렉토리 이동
cd ~/.ladspa
```

**아키텍처**: x86_64 인 경우 

```
# DeepFilter LADSPA 플러그인 다운로드 (v0.5.6, x86_64)
wget https://github.com/Rikorose/DeepFilterNet/releases/download/v0.5.6/libdeep_filter_ladspa-0.5.6-x86_64-unknown-linux-gnu.so
```

**아키텍처**: aarch64 인 경우 

```
# DeepFilter LADSPA 플러그인 다운로드 (v0.5.6, aarch64)
wget https://github.com/Rikorose/DeepFilterNet/releases/download/v0.5.6/libdeep_filter_ladspa-0.5.6-aarch64-unknown-linux-gnu.so
```

```
# 파일 크기 확인 (약 50MB)
ls -lh libdeep_filter_ladspa-0.5.6-*.so

# 실행 권한 부여
chmod +x libdeep_filter_ladspa-0.5.6-*.so

# LADSPA 환경 변수 설정
export LADSPA_PATH=$HOME/.ladspa
echo 'export LADSPA_PATH=$HOME/.ladspa' >> ~/.bashrc
source ~/.bashrc
```

**플러그인 확인:**

```
# LADSPA SDK 설치 (없는 경우)
sudo apt install ladspa-sdk

# 플러그인 목록 확인
listplugins | grep -i deep
```

**예상 출력:**
```
DeepFilter Mono (7843795/deep_filter_mono)
DeepFilter Stereo (7843796/deep_filter_stereo)
```

# 3단계: PipeWire-Pulse 설치

PulseAudio를 PipeWire-Pulse로 교체

```
# PipeWire-Pulse 설치
sudo apt install pipewire-pulse

# 기존 PulseAudio 비활성화
# 비활성화를 하지 않으면, 나중에 기기가 deepfilter와 기존 마이크를 왔다갔다 함
systemctl --user mask pulseaudio.service pulseaudio.socket

# PulseAudio 프로세스 종료
pulseaudio -k
killall pulseaudio 2>/dev/null

# PipeWire-Pulse 활성화
systemctl --user unmask pipewire-pulse.service pipewire-pulse.socket
systemctl --user enable pipewire-pulse.service pipewire-pulse.socket
systemctl --user start pipewire-pulse.service
```

# 4단계: WirePlumber 설치
WirePlumber는 PipeWire의 세션 매니저로, ALSA 오디오 장치를 자동으로 관리
```
# WirePlumber 설치
sudo apt install wireplumber

# systemd 심볼릭 링크 충돌 해결
sudo rm -f /etc/systemd/user/pipewire-session-manager.service

# systemd 리로드
sudo systemctl daemon-reload
systemctl --user daemon-reload

# WirePlumber 활성화
systemctl --user enable wireplumber.service
```

# 5단계: 필터 체인 설정

PipeWire 필터 체인 설정 파일을 생성

## 5-1. 오디오 입력 장치 확인

먼저 사용할 마이크 장치의 정확한 이름을 확인

```
# 오디오 입력 장치 목록 확인
pactl list sources short
```

**출력 예시:**

```
54  alsa_input.usb-Harman_International_Inc_JBL_Quantum_Stream-00.mono-fallback
56  alsa_input.pci-0000_00_1f.3.analog-stereo
```

## 5-2. 설정 파일 생성

```
# 폴더 생성
mkdir -p ~/.config/pipewire/


# 설정 파일 생성
nano ~/.config/pipewire/filter-chain.conf
```

**다음 내용을 복사해서 붙여넣으세요:**

```
context.properties = {
    log.level = 2
}

context.spa-libs = {
    audio.convert.* = audioconvert/libspa-audioconvert
    support.*       = support/libspa-support
}

context.modules = [
    { name = libpipewire-module-rt
        args = {
            nice.level = -11
        }
        flags = [ ifexists nofail ]
    }
    { name = libpipewire-module-protocol-native }
    { name = libpipewire-module-client-node }
    { name = libpipewire-module-adapter }
    { name = libpipewire-module-link-factory }

    { name = libpipewire-module-filter-chain
        args = {
            node.description = "DeepFilter Noise Canceling Source"
            media.name = "DeepFilter Noise Canceling Source"
            filter.graph = {
                nodes = [
                    {
                        type = ladspa
                        name = deepfilter
                        plugin = /home/YOUR_USERNAME/.ladspa/libdeep_filter_ladspa-0.5.6-x86_64-unknown-linux-gnu.so
                        label = deep_filter_mono
                        control = {
                            "Attenuation Limit (dB)" = 80
                        }
                    }
                ]
            }
            audio.position = [ MONO ]
            capture.props = {
                node.name = "deepfilter_capture"
                node.passive = true
                audio.rate = 48000
                stream.dont-remix = true
                node.target = "YOUR_MICROPHONE_DEVICE_NAME"
            }
            playback.props = {
                node.name = "DeepFilterMic"
                media.class = "Audio/Source"
                audio.rate = 48000
            }
        }
    }
]

```

**⚠️ 중요: 다음 두 곳을 수정해야 합니다:**

1. 플러그인 경로
2. 마이크 장치 이름 

```
plugin = /home/YOUR_USERNAME/.ladspa/libdeep_filter_ladspa-0.5.6-x86_64-unknown-linux-gnu.so
```

```
node.target = "alsa_input.usb-Harman_International_Inc_JBL_Quantum_Stream-00.mono-fallback"
```

# 6단계: systemd 서비스 생성

부팅 시 자동으로 DeepFilter가 시작되도록 systemd 서비스 생성

```
# systemd 사용자 서비스 디렉토리 생성
mkdir -p ~/.config/systemd/user/

# 서비스 파일 생성
nano ~/.config/systemd/user/pipewire-filter-chain.service
```

**다음 내용을 복사해서 붙여넣으세요:**

```
[Unit]
Description=PipeWire Filter Chain for DeepFilter
After=pipewire.service pipewire-pulse.service wireplumber.service
Requires=pipewire.service

[Service]
Type=simple
ExecStart=/usr/bin/pipewire -c %h/.config/pipewire/filter-chain.conf
Restart=on-failure
RestartSec=3

[Install]
WantedBy=default.target
```

# 7단계: 서비스 활성화

모든 서비스를 올바른 순서로 시작

```
# systemd 데몬 리로드
systemctl --user daemon-reload

# pipewire-filter-chain 서비스 활성화 (부팅 시 자동 시작)
systemctl --user enable pipewire-filter-chain.service

# 모든 서비스를 올바른 순서로 재시작
systemctl --user restart pipewire.service
sleep 2
systemctl --user start wireplumber.service
sleep 2
systemctl --user restart pipewire-pulse.service
sleep 2
systemctl --user start pipewire-filter-chain.service
sleep 2


```

# 8단계: 확인 및 테스트

## 8-1. 서비스 상태 확인

```
# WirePlumber 상태
systemctl --user status wireplumber.service

# pipewire-filter-chain 상태
systemctl --user status pipewire-filter-chain.service
```

## 8-2. 오디오 장치 확인

```
# 오디오 카드 목록
pactl list cards short

# 오디오 소스 목록
pactl list sources short
```

**예상 출력 (pactl list sources short):**

```
54  alsa_input.usb-Harman_International_Inc_JBL_Quantum_Stream-00.mono-fallback  PipeWire  ...
82  DeepFilterMic  PipeWire  float32le 1ch 48000Hz  SUSPENDED
```

✅ **DeepFilterMic이 보이면 성공입니다!**

## 8-3. 녹음 테스트

```
# 5초간 녹음 (DeepFilter를 통해)
arecord -D default -f S16_LE -r 48000 -c 1 -d 5 test_deepfilter.wav

# 녹음 중 소리를 내고, 배경 소음(키보드, 팬 등)도 함께 발생시켜보세요

# 재생
aplay test_deepfilter.wav

```

# 9단계: 기본 마이크 설정

DeepFilterMic을 시스템 기본 마이크로 설정

```
# DeepFilterMic을 기본 마이크로 설정
pactl set-default-source DeepFilterMic

# 확인
pactl info

# 또는
pactl info | grep "기본 소스"
```

---

# 참고: 대안 방법

⚠️ **주의:** 이 방법은 PipeWire 버전에 따라 자동 로드가 안 될 수 있습니다.
위의 기본 방법(systemd 서비스)을 권장합니다.

### 설정 파일 위치
- `~/.config/pipewire/filter-chain.conf.d/deepfilter-mono-source.conf`
- `~/.config/pipewire/pipewire.conf.d/50-alsa-config.conf`

### 주의사항
- 마이크 장치 이름을 본인 환경에 맞게 수정
- PipeWire 0.3.48에서는 수동 서비스 설정 필요할 수 있음


# 필터 적용 전후 차이 

[필터 적용 전](aeirobot_dialogue/audio_setup/demo_audio_files/before_deepfilter.wav)

[필터 적용 후](aeirobot_dialogue/audio_setup/demo_audio_files/after_deepfilter.wav)