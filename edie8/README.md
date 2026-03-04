# EDIE8

<a href="https://arobot4all.com">
  <img src="./edie_system_setting/sys_img/edie_readme.png" alt="arobot4all" style="max-width: 100%; height: auto;"/>
</a>

### Dev Enviroment

✅ Ubuntu 22.04 (Jammy Jellyfish)  
✅ ROS2 (Humble)

### Dependencies

<details>
<summary>펼쳐보기</summary>

[의존성 자동 설치 (One-Touch Script)](./edie_system_setting/README.md)

#### 필요 레파지토리 

```
git clone -b feature/edie_vins_fusion git@github.com:HERoEHS/edie8_parameters.git
git clone -b develop git@github.com:HERoEHS/edie8_simulation.git
git clone -b feature/edie git@github.com:HERoEHS/aeirobot_toolbox.git
```
#### SDL2-Image
```
sudo apt install -y libsdl2-image-dev
```

#### yaml-cpp
```
wget https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.6.3.zip # (v0.6.3)
unzip yaml-cpp-0.6.3.zip
cd yaml-cpp-yaml-cpp-0.6.3
mkdir build
cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=ON ..
make
sudo make install
```

#### jsoncpp
```
sudo apt update
sudo apt install libjsoncpp-dev
```
#### OpenCV
```
cd ~/
git clone https://github.com/opencv/opencv.git
cd opencv && mkdir build && cd build

sudo apt-get install python3-pip
pip3 install numpy
```
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
-D PYTHON3_EXECUTABLE=$(which python3) \
-D PYTHON3_INCLUDE_DIR=$(python3 -c "from sysconfig import get_paths as gp; print(gp()['include'])") \
-D PYTHON3_NUMPY_INCLUDE_DIRS=/usr/lib/python3/dist-packages/numpy/core/include \
-D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
-D PYTHON3_LIBRARY=$(python3 -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))") \
-D WITH_GTK=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
..
```
```
make -j2
sudo make install
```
```
pip3 install opencv-contrib-python==4.8.0.74
```
#### BehaviorTree.CPP package
```
sudo apt-get update
sudo apt-get install -y libzmq3-dev

cd
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j
sudo make install
```

#### joy & teleop_joy
```
sudo apt install ros-humble-joy
```
```
sudo apt install ros-humble-teleop-twist-joy
```

#### localization
```
sudo apt install libgeographic-dev
```

```
sudo apt install libgpiod-dev
```


</details>

### Execute command
<details>
<summary>펼쳐보기</summary>

- 노드 전체 실행
```
ros2 launch edie_bringup bringup_robot.launch.py 
```

- 테스트 GUI 실행
```
ros2 run edie8_test edie8_test_gui
```

- Gazebo 시뮬레이션 실행
```
ros2 launch edie_bringup launch_gazebo.launch.py
```

- (옵션_1) 키보드 주행
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/edie8/diff_drive_controller/cmd_vel_unstamped
```

- (옵션_2) 조이스틱 주행
```
ros2 launch edie8_joy telelop_joy.launch.py
```
</details>

### Topic and Service names (datatype)
<details>
<summary>펼쳐보기</summary>

#### Emotion Topic
```
/edie8/emotion/action_index (std_msgs/msg/UInt8)
/edie8/emotion/emotion_done (std_msgs/msg/Bool)
/edie8/emotion/sound_index (std_msgs/msg/UInt8)
/edie8/emotion/sound_done (std_msgs/msg/Bool)
/edie8/emotion/display_index (std_msgs/msg/UInt8)
/edie8/emotion/display_done (std_msgs/msg/Bool)
/edie8/emotion/motion_index (std_msgs/msg/UInt8)
/edie8/emotion/motion_done (std_msgs/msg/Bool)
```

#### Navigation Topic
```
/edie8/diff_drive_controller/cmd_vel_unstamped (geometry_msgs/msg/Twist)
/edie8/navigation/waypoint_number (std_msgs/msg/Int32)
/edie8/navigation/goal_pose (geometry_msgs/msg/Twist)
/edie8/navigation/path (nav_msgs/msg/Path)
```

#### Sensors Topic
```
/edie8/sensor/bottom/laser (std_msgs/msg/Int16MultiArray)
/edie8/sensor/front/laser (std_msgs/msg/Int16MultiArray)
/edie8/sensor/fsr (std_msgs/msg/Int16MultiArray)
/edie8/sensor/imu (sensor_msgs/msg/Imu)
```

#### Service
```
/edie8/action_ready (edie_msgs/srv/CheckReady)
```

#### Action
```
```
</details>

### Submodule
<details>
<summary>펼쳐보기</summary>

#### 환경변수 설정
```
export ROS2_WS=ros2_ws
export ROS_WS=$HOME/$ROS2_WS
```
#### Submodule 최신화/가져오기 
```
git submodule update --init --recursive
```
</details>

### Image file(.img) 추출
<details>
<summary>펼쳐보기</summary>

#### Ubuntu 환경에서 Micro SD 카드 인식됨.

##### Step 1. SD 카드 이미지 추출
```
0. SD 리더기 통해 Micro SD 카드를 컴퓨터에 연결합니다.
1. "디스크"를 검색하여 프로그램을 실행합니다.
2. 왼쪽에서 micro SD 카드를 선택합니다.
3. 오른쪽 위의 메뉴(세 개 점)를 클릭하고 "디스크 이미지로 만들기…"를 선택합니다.
4. 저장 위치와 파일 이름을 선택하고 "저장"을 클릭하면 이미지 파일이 생성됩니다.
```

##### Step 2. 추출된 이미지 SD 카드로 옮기기 
```
# SD 리더기 통해 빈 SD 카드 컴퓨터에 연결
# 장치 검색 
sudo fdisk -l
# Disk /dev/sda: 59.49 GiB, 63864569856 bytes, 124735488 sectors

# 이미지 파일을 sd 카드에 쓰기
sudo dd if=[이미지 파일] of=[복사할 장치] bs=[블록 사이즈] status=[진행 옵션]
# 이미지 파일: 경로 및 파일 이름, 확장자 명시
# 복사할 장치: fdisk를 통해 알아낸 장치
# 블록 사이즈: 그냥 8M 주자. 작으면 추출 시간이 오래 걸리고 크면 뻑날지도??
# 진행 옵션: status=progress 옵션을 주면 진행 상태를 알려줌
```
</details>

### VINS-fusion
<details>
<summary>펼쳐보기</summary>

#### 예제 명령어
#### 
```
cw
ros2 launch vins vins_rviz.launch.xml 
ros2 bag play src/edie8/VINS-Fusion-ROS2-humble/data/MH_01_easy/
ros2 run vins vins_node src/edie8/VINS-Fusion-ROS2-humble/config/euroc/euroc_mono_imu_config.yaml 
python3 src/edie8/VINS-Fusion-ROS2-humble/vins/launch/frame_id_add.py (급하게......)
```
</details>
