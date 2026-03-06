from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aeirobot_llm'

# Get all resource files, excluding directories
resource_files = [f for f in glob('resource/*') if os.path.isfile(f)]
prompt_files = [f for f in glob('resource/prompts/*') if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), resource_files),
        (os.path.join('share', package_name, 'resource/prompts'), prompt_files),
    ],
    install_requires=['setuptools', 'rclpy', 'python-dotenv'],
    zip_safe=True,
    maintainer='yh',
    maintainer_email='jeremykim95@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_main = aeirobot_llm.agent_main:main',
            'llm_main = aeirobot_llm.llm_main:main',
        ],
    },
)
