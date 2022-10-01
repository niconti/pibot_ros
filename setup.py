from setuptools import setup

package_name = 'pibot_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicola Conti',
    maintainer_email='continicola89@gmail.com',
    description='ROS nodes for PiBot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_camera = pibot_ros.teleop_camera:main',
        ],
    },
)
