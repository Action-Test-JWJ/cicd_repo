# 'emotion' 감지 기능[심화]
## 상세 설명
* ~~카메라 센서~~ **'edie8/vision/image_raw' 토픽 subscription** 통한 얼굴 인식 시 **7개 감정들 중** 특정 감정에 대한 감정 깊이('Arousal')의 probability 값 tanh 정규화
    * **“Neutral", "Anger", "Happiness", "Surprise", "Disgust", "Sadness", "Fear”**
* 정규화된 Arousal probability 값을 길이 7 벡터의 인식된 특정 감정 index에 업데이트 및 publish

---

## 실행 방법
```
# Step 0. '2803_vgg16_full.model' 경로 확인
# 코드에 저장된 경로는 './ros2_ws/src/edie8/edie_vora/edie_emotion_detection/edie_emotion_detection/model'

# Step 1. 환경설정
sudo apt update
sudo apt install pkg-config build-essential ninja-build cmake
sudo apt install libx11-dev libgtk2.0-dev python3-tk
# sudo apt install libavcodec-dev libavformat-dev libswscale-dev libjpeg-dev libopencv-dev libpng-dev(개발시에만 필요)
sudo apt update && sudo apt install -y python3-dev python3-pip
pip install --upgrade wheel setuptools pip

# Step 2. 패키지 디렉토리 내부에 환경설정 파일실행
# 아래 명령어실행중 오류 발생시 오류난 라이브러리 직접설치 version은 큰 상관없는것으로 보임
pip install -r requirements.txt

# Step 3. 빌드 및 패키지 ros2 환경 불러오기 (터미널 1)
colcon build
source install/local_setup.bash

# (노트북 시) Step 4-1. 가상환경 활성화 (터미널 1)
cd venv
source bin/activate

# Step 4-2. gti, VORA2803 권한설정 및 ros2  환경 불러오기
cd ./ros2_ws/src/edie8/edie_vora/edie_emotion_detection/edie_emotion_detection
source SourceME.env
source install/local_setup.bash

# Step 5. Publish 노드 실행 (터미널 1)
ros2 run edie_camera edie_camera_pub --ros-args --params-file /home/vraptor/ros2_ws/src/edie8/edie_sensor/edie_camera/config/config.yaml

# Step 6. Publish & Subscriber 노드 실행 (터미널 2)
ros2 run edie_emotion_detection emotion_pub

# Step 7. topic 정보 확인 (터미널 3)
ros2 topic echo /edie8/vision/normalized_emotion_depth_array
```
## 애러 해결방법
```
# 1. raise GtiDeviceException(GtiDeviceExceptionCause.NO_AVAILABLE_DEVICES_FOUND)
## .model 파일 있나 확인!!!

# 2. KeyError: 'GTISDKPATH'
export GTISDKPATH=/home/vraptor/ros2_ws/src/edie8/edie_vora/edie_emotion_detection/edie_emotion_detection

# 3. No module named 'gtilib'
## 에러 발생한 터미널에서 'edie_emotion_detection' 패키지 경로 추가
export PYTHONPATH=$PYTHONPATH:$HOME/ros2_ws/src/edie8/edie_vora/edie_emotion_detection/edie_emotion_detection

# 4.No module named '_dlib_pybind11'
## 설치 유무 확인
pip3 show dlib

## 없으면 requirements.txt에서 버전 확인 후 설치
pip3 install dlib==19.24.2

```
