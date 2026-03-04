import os
from glob import glob
from setuptools import setup

package_name = 'edie_aruco'

if __name__ in ['__main__', 'builtins']:
    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
            (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
            (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='JAEWOOK6488',
        maintainer_email='m6488j@arobot4all.com',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'aruco_detection_node = edie_aruco.aruco_detection_node:main',
                'aruco_detection_only_pose_node = edie_aruco.aruco_detection_only_pose_node:main',
                'rviz_marker_visualize = edie_aruco.rviz_marker_visualize:main',
            ],
        },
    )
