from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'edie_emotion_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the model file in the package data
        (os.path.join('share', package_name, 'model'), 
        glob('edie_emotion_detection/model/2803_vgg16_full.model')),
        # Include the config file in the package data
        (os.path.join('share', package_name, 'config'), 
        glob('edie_emotion_detection/config/landmarks.config')),
    ],

    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'sensor_msgs',
        'geometry_msgs',
        'opencv-python',
        'cv_bridge',
        'image_transport',
        'aeirobot_toolbox',
        "edie_msgs",
    ],
    
    zip_safe=True,
    maintainer='edie',
    maintainer_email='edie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emotion_pub = edie_emotion_detection.edie_emotion_detection_node:main',
            'reader = edie_emotion_detection.gyurinet.gtilib.reader:main',
            'util = edie_emotion_detection.gyurinet.gtilib.util:main',
            'detector = edie_emotion_detection.detector:main',
            'demo = edie_emotion_detection.demo:main',
        ],
    },
)
