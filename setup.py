import os
from glob import glob
from setuptools import setup

package_name = 'insta360x4_webcam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # 添加 data_files 来安装 package.xml 和其他资源文件
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 这行会自动包含 launch 目录下的所有启动文件
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    # ROS依赖项由 package.xml 管理，不应放在这里
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Longxiaoze',
    maintainer_email='longxiaoze666@gmail.com',
    description='Insta360 X4 Webcam Node to capture and publish images to ROS 2',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            # 这一行定义了 'ros2 run insta360x4_webcam insta360x4_node' 命令
            'insta360x4_node = insta360x4_webcam.insta360x4_node:main',
        ],
    },
)