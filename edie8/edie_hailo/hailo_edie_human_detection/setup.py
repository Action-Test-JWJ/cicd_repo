from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hailo_edie_human_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # install model and common assets for runtime
        (os.path.join('share', package_name, 'model'), ['model/yolov8n.hef']),
        # common python modules (so runtime import via sys.path works)
        (os.path.join('share', package_name, 'common'), [
            '../common/coco.txt',
            '../common/hailo_inference.py',
            '../common/toolbox.py',
        ]),
        (os.path.join('share', package_name, 'common', 'tracker'), glob('../common/tracker/*.py')),
        # runtime config
        (os.path.join('share', package_name), ['config.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rp5',
    maintainer_email='rp5@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hailo_edie_human_detection = hailo_edie_human_detection.hailo_edie_human_detection_node:main',
        ],
    },
)
