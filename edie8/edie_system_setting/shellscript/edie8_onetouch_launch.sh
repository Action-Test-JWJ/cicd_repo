#!/bin/bash

sleep 10 # (부팅 대기 margin)
source /opt/ros/humble/setup.bash
source /home/vraptor/ros2_ws/install/setup.bash 
sleep 2
sudo alsa force-reload &
sleep 3
ros2 launch edie_bringup bringup_robot.launch.py & 
sleep 20
ros2 launch hailo_edie_human_detection edie_human_detection.launch.py &
sleep 10
ros2 launch ml_edie_emotion_detection edie_emotion_detection.launch.py &
sleep 10
ros2 launch edie_behavior edie_behavior.launch.py &
sleep 5
ros2 topic pub --once /edie8/gui/operation_mode std_msgs/msg/String '{data: "follow"}' &
sleep 5
ros2 run joystick_ros2 joy_udp_receiver &
sleep 3
wait
