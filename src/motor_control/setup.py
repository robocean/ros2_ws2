from setuptools import setup, find_packages

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # 패키지 자동 탐색
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='ROS 2 motor control package with Dynamixel SDK integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = motor_control.talker_motor:main',
            'listener = motor_control.listener_motor:main',
            'motor_control_node = motor_control.motor_control_node:main',
        ],
    },
)

