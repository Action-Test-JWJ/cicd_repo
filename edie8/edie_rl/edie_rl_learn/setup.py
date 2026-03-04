from setuptools import setup
package_name = 'edie_rl_learn'
submodules = "edie_rl_learn/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eking',
    maintainer_email='eking158@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'edie_rl_ppo_main_node = edie_rl_learn.ppo_ros_main:main',
            'edie_rl_ppo_load_node = edie_rl_learn.ppo_ros_load:main',
            'edie_rl_learn = edie_rl_learn.edie_rl_learn_node:main',
        ],
    },
)
