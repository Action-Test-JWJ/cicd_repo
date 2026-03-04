from setuptools import setup
import os
from glob import glob

package_name = 'edie_node_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/ui', glob(os.path.join('ui', '*'))),
        ('share/' + package_name + '/log', glob(os.path.join('log', '*.txt'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name + '/param', glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwj',
    maintainer_email='jwj@arobot4all.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'edie_node_register = edie_node_manager.node_register_main:main',
            'edie_node_manager = edie_node_manager.node_manager_main:main',
        ],
    },
)
