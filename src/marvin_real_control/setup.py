from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'marvin_real_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tjzn-zwt',
    maintainer_email='tjzn-zwt@example.com',
    description='Marvin机器人真实控制包，用于连接实际机器人硬件',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marvin_real_controller = marvin_real_control.marvin_real_controller:main',
            'marvin_moveit_bridge_demo = marvin_real_control.marvin_moveit_bridge_demo:main',
            'jointstate_to_marvin_cmd = marvin_real_control.jointstate_to_marvin_cmd:main',
            'trajectory_to_marvin_cmd = marvin_real_control.trajectory_to_marvin_cmd:main',
        ],
    },
)
