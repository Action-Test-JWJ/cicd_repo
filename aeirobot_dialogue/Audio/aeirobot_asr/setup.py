from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aeirobot_asr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['aeirobot_asr', 'aeirobot_asr.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuhyun Kim',
    maintainer_email='jeremykim95@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asr_main = aeirobot_asr.asr_main:main',
            'record = aeirobot_asr.record:main',
        ],
    },
)
