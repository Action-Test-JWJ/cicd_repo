from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'ml_edie_emotion_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pkl'))
    ],
    install_requires=[
        'setuptools',
        'scikit-learn', 
        'mediapipe',
        'rclpy',
        'std_msgs',
        'sensor_msgs',
        'geometry_msgs',
        'opencv-python',
        'cv_bridge',
        'aeirobot_toolbox',
        'edie_msgs',
    ],
    zip_safe=True,
    maintainer='higony',
    maintainer_email='heagon72@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'edie_emotion_detection_node = ml_edie_emotion_detection.edie_emotion_detection_node:main',
        ],
    },
)
