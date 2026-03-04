from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'edie_mobile_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'edie_mobile_action_node = edie_mobile_action.edie_mobile_action_node:main',
        ],
    },
)
