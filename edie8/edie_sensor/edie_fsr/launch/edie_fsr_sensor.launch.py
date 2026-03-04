from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    # 필터링 관련 파라미터
    min_cutoff_arg = DeclareLaunchArgument(
        'min_cutoff', default_value='5.0', description='Minimum cutoff frequency')
    beta_arg = DeclareLaunchArgument(
        'beta', default_value='0.0', description='Beta parameter for OneEuroFilter')
    dcutoff_arg = DeclareLaunchArgument(
        'dcutoff', default_value='1.0', description='Derivative cutoff frequency')
    fsr_hz_arg = DeclareLaunchArgument(
        'fsr_hz', default_value='50.0', description='FSR Hz')
    rise_time_arg = DeclareLaunchArgument(
        'rise_time', default_value='0.01', description='Interpolation rise time[sec]. 0.1~0.2 새로 세게 쓰다듬으면 → 빠르게 올라가기')
    fall_time_arg = DeclareLaunchArgument(
        'fall_time', default_value='0.35', description='Interpolation fall time[sec]. 1~2초 천천히만 내려가서 “기억”이 남기')

    # 상태머신 관련 파라미터
    noise_threshold_arg = DeclareLaunchArgument(
        'noise_threshold', default_value='80.0', description='Touch detection threshold (filtered delta)')
    fsr_buffer_size_arg = DeclareLaunchArgument(
        'fsr_buffer_size', default_value='250', description='FSR buffer size. 50샘플 ≒ 1초(50Hz)')
    # StrongTouch -> WeakTouch
    strong_to_weak_hold_time_sec_arg = DeclareLaunchArgument(
        'strong_to_weak_hold_time_sec', default_value='3.0', description='StrongTouch → WeakTouch 로 내려가는 데 필요한 유지 시간')
    strong_to_weak_band_ratio_arg = DeclareLaunchArgument(
        'strong_to_weak_band_ratio', default_value='0.3', description='대표값 기준으로 “비슷한 힘”이라고 볼 구간 (band). 대표값의 ±x*100 %')
    strong_to_weak_min_arg = DeclareLaunchArgument(
        'strong_to_weak_min', default_value='50.0', description='최소 margin (filtered 데이터 기준). 5 -> 50')
    # WeakTouch -> StrongTouch
    weak_to_strong_hold_time_sec_arg = DeclareLaunchArgument(
        'weak_to_strong_hold_time_sec', default_value='0.05', description='WeakTouch → StrongTouch 로 올라가는 데 필요한 유지 시간')
    weak_to_strong_band_ratio_arg = DeclareLaunchArgument(
        'weak_to_strong_band_ratio', default_value='0.4', description='대표값보다 x*100 % 이상 크면 StrongTouch')
    weak_to_strong_min_arg = DeclareLaunchArgument(
        'weak_to_strong_min', default_value='100.0', description='최소 margin (filtered 데이터 기준). 15 -> 150')
    weak_to_strong_max_arg = DeclareLaunchArgument(
        'weak_to_strong_max', default_value='180.0', description='최대 margin (filtered 데이터 기준). 150')

    node = Node(
        package='edie_fsr',
        executable='edie_fsr_sensor_node',
        name='edie_fsr_sensor_node',
        output='screen',
        parameters=[{
            'min_cutoff': LaunchConfiguration('min_cutoff'),
            'beta': LaunchConfiguration('beta'),
            'dcutoff': LaunchConfiguration('dcutoff'),
            'fsr_hz': LaunchConfiguration('fsr_hz'),
            'rise_time': LaunchConfiguration('rise_time'),
            'fall_time': LaunchConfiguration('fall_time'),
            'noise_threshold': LaunchConfiguration('noise_threshold'),
            'fsr_buffer_size': LaunchConfiguration('fsr_buffer_size'),
            'strong_to_weak_hold_time_sec': LaunchConfiguration('strong_to_weak_hold_time_sec'),
            'strong_to_weak_band_ratio': LaunchConfiguration('strong_to_weak_band_ratio'),
            'strong_to_weak_min': LaunchConfiguration('strong_to_weak_min'),
            'weak_to_strong_hold_time_sec': LaunchConfiguration('weak_to_strong_hold_time_sec'),
            'weak_to_strong_band_ratio': LaunchConfiguration('weak_to_strong_band_ratio'),
            'weak_to_strong_min': LaunchConfiguration('weak_to_strong_min'),
            'weak_to_strong_max': LaunchConfiguration('weak_to_strong_max'),
        }]
    )

    return LaunchDescription([
        min_cutoff_arg,
        beta_arg,
        dcutoff_arg,
        fsr_hz_arg,
        rise_time_arg,
        fall_time_arg,
        noise_threshold_arg,
        fsr_buffer_size_arg,
        strong_to_weak_hold_time_sec_arg,
        strong_to_weak_band_ratio_arg,
        strong_to_weak_min_arg,
        weak_to_strong_hold_time_sec_arg,
        weak_to_strong_band_ratio_arg,
        weak_to_strong_min_arg,
        weak_to_strong_max_arg,
        node,
    ])

