from setuptools import setup

package_name = 'waypoint'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/waypoint_launch.py']),  # Launch 파일 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='ROS 2 package for waypoint navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_controller = waypoint.waypoint_controller:main',  # 로봇 제어 노드
            'status_monitor = waypoint.status_monitor:main',            # 상태 모니터 노드
        ],
    },
)

