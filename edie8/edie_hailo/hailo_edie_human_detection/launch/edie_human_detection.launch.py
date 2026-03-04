from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from pathlib import Path

def generate_launch_description():
    # 설치 공간에서도 안전하게: 패키지 share 디렉터리 기준으로 경로 지정
    net_default = PathJoinSubstitution([
        FindPackageShare('hailo_edie_human_detection'), 'model', 'yolov8n.hef'])
    labels_default = PathJoinSubstitution([
        FindPackageShare('hailo_edie_human_detection'), 'common', 'coco.txt'])
    
    # ---- Launch args (원하면 CLI로 바꿔치기 가능) ----
    net_arg       = DeclareLaunchArgument('net', default_value=net_default, 
                                          description='Path to HEF model')
    labels_arg    = DeclareLaunchArgument('labels', default_value=labels_default, 
                                           description='Path to labels txt')
    batch_arg     = DeclareLaunchArgument('batch_size', default_value='32', # 32 or 63 
                                           description='Batch size for inference')
    res_arg       = DeclareLaunchArgument('resolution', default_value='sd', 
                                           description="Choose input resolution: 'sd' (640x480), 'hd' (1280x720), or 'fhd' (1920x1080).")  # sd|hd|fhd
    track_arg     = DeclareLaunchArgument('track', default_value='true', 
                                           description='Enable object tracking')
    fps_arg       = DeclareLaunchArgument('show_fps', default_value='false', 
                                           description='Show FPS')
    show_win_arg = DeclareLaunchArgument('show_window', default_value='false', 
                                         description='Show OpenCV window (true/false)')
    lat_arg = DeclareLaunchArgument('measure_latency', default_value='false', 
                                    description='Measure latency using perf_counter_ns')

    node = Node(
        package='hailo_edie_human_detection',
        executable='hailo_edie_human_detection',   # setup.py의 entry_point와 일치해야 함
        name='hailo_edie_human_detection',
        output='screen',
        parameters=[{
            'net':        LaunchConfiguration('net'),
            'labels':     LaunchConfiguration('labels'),
            'batch_size': LaunchConfiguration('batch_size'),
            'resolution': LaunchConfiguration('resolution'),
            'track':      LaunchConfiguration('track'),
            'show_fps':   LaunchConfiguration('show_fps'),
            'show_window': LaunchConfiguration('show_window'),
            'measure_latency': LaunchConfiguration('measure_latency'),
        }],
    )

    return LaunchDescription([
        net_arg, 
        labels_arg, 
        batch_arg, 
        res_arg, 
        track_arg, 
        fps_arg,
        show_win_arg,
        lat_arg,
        node
    ])
