### 의존성 자동 설치 (One-Touch Script)

```
cd ${ROS_WS}/src/edie8/edie_system_setting/shellscript/

chmod +x edie8_dependency.sh

./edie8_dependency.sh   
```

# -------------------- 로봇 --------------------------

### 에디 vnc 세팅 

```
cd ${ROS_WS}/src/edie8/edie_system_setting/shellscript/

chmod +x edie8_vnc_setup.sh

./edie8_vnc_setup.sh

```

### 런치파일 원터치 스크립 등록
```
cd ${ROS_WS}/src/edie8/edie_system_setting/service/

sudo cp edie8_ros2_startup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable edie8_ros2_startup.service
```

### 마이크 등록
```
cd ${ROS_WS}/src/edie8/edie_system_setting/service/

sudo cp edie8_speaker_gpio.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable edie8_speaker_gpio.service
sudo systemctl enable edie8_uart_start.service
```
### 마이크 시작
```
sudo systemctl start edie8_speaker_gpio.service edie8_uart_start.service
```

### 마이크 상태 확인
```
sudo systemctl status edie8_speaker_gpio.service
```
### 마이크 삭제
```
sudo systemctl stop edie8_speaker_gpio.service
sudo systemctl disable edie8_speaker_gpio.service
sudo rm /etc/systemd/system/edie8_speaker_gpio.service
```