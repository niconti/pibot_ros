import os
from glob import glob
from setuptools import setup

package_name = 'zumo_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share/ament_index/resource_index/packages'), ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*config.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicola Conti',
    maintainer_email='continicola89@gmail.com',
    description='ROS nodes for Zumo Robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors_nvidia = zumo_ros.motors_nvidia:main',
            'teleop_camera = zumo_ros.teleop_camera:main',
        ],
    },
)
