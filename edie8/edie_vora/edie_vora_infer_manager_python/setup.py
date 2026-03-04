from setuptools import find_packages, setup

package_name = 'edie_vora_infer_manager_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='higony',
    maintainer_email='heagon72@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vora_infer_manager_python_node = edie_vora_infer_manager_python.manager_node:main',
        ],
    },
)
