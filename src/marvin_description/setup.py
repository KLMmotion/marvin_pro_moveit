from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'marvin_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 修复路径：使用相对路径而不是绝对路径
        (os.path.join('share', package_name, 'urdf'), glob("urdf/*.xacro")),
        (os.path.join('share', package_name, 'urdf'), glob("urdf/*.rviz")),
        (os.path.join('share', package_name, 'launch'), glob("launch/*.launch.py")),
        # 添加 meshes 文件安装
        (os.path.join('share', package_name, 'meshes/m6'), glob("meshes/m6/*.STL")),
        (os.path.join('share', package_name, 'meshes/base'), glob("meshes/base/*.STL")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nanyun',
    maintainer_email='wzhou3943@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
