import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from ament_index_python.packages import get_package_share_directory
from os.path import join, expanduser

def generate_launch_description():
    # 파라미터 설정
    # usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.2/Isaac/Environments/Grid/default_environment.usd"
    # usd_path = join(get_package_share_directory('isaac_ros2_scripts'), 'meshes/USD/default_stage.usd')
    # usd_path = join(get_package_share_directory('edie8_isaac_sim'), 'asset/Grid/default_environment.usd')
    usd_path = join(get_package_share_directory('edie8_isaac_sim'), 'asset/SimpleRoom/simple_room.usd')
    
    
    fps = 60.0
    real_fps = fps
    time_steps_per_second = 600.0

    isaac_path = join(expanduser("~"), 'isaacsim_4.2.0')

    python_script = join(isaac_path, 'python.sh')
    if not os.path.isfile(python_script):
        raise FileNotFoundError(f"Isaac Sim python.sh not found at {python_script}")

    start_script = join(get_package_share_directory('isaac_ros2_scripts'), 'start_sim.py')

    # 환경 변수 설정
    fastdds_config = join(get_package_share_directory('isaac_ros2_scripts'), 'config/fastdds.xml')
    env = {'FASTRTPS_DEFAULT_PROFILES_FILE': fastdds_config}

    command = [
        "bash", python_script, start_script, usd_path,
        str(fps), str(time_steps_per_second), str(real_fps), "False"
    ]

    return LaunchDescription([
        LogInfo(msg=f"Launching Isaac Sim with command: {' '.join(command)}"),
        ExecuteProcess(
            cmd=command,
            shell=False,
            output='screen',
            additional_env=env
        )
    ])
