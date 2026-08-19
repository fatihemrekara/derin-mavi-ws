import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'zed_pose_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='derinmavi',
    maintainer_email='derinmavi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_to_local_node = zed_pose_test.gps_to_local_node:main',
            'local_waypoints_node = zed_pose_test.local_waypoints_node:main',
            'route_planner = zed_pose_test.route_planner_node:main',
            'square_route_planner = zed_pose_test.square_route_planner:main',
            'straight_route_planner = zed_pose_test.straight_route_planner:main',
            'zed_pose_logger = zed_pose_test.zed_pose_logger_ros2:main',
            'path_follower = zed_pose_test.path_follower_node:main',
            'fake_robot_sim = zed_pose_test.fake_robot_sim:main',
            'underwater_physics_sim = zed_pose_test.underwater_physics_sim:main',
            'fake_zed_camera = zed_pose_test.fake_zed_camera:main',
            'fake_imu_sim = zed_pose_test.fake_imu_sim:main',
            'sensor_fusion_ekf = zed_pose_test.sensor_fusion_ekf:main',
            'nozed_test = zed_pose_test.nozed_test:main',
            'zed_to_mavros_bridge = zed_pose_test.zed_to_mavros_bridge:main',
            'videotask = zed_pose_test.videotask:main',
            'video_gorevi = zed_pose_test.video_gorevi:main',
            'MiniRov_cam = zed_pose_test.MiniRov_cam:main',
            'path_follower_steps = zed_pose_test.path_follower_steps:main',
            'station_gui = zed_pose_test.station_gui:main',
            'return_route_planner = zed_pose_test.return_route_planner:main',
            'sensor_comparison_logger = zed_pose_test.sensor_comparison_logger:main',
            'ekf_sensor_evaluation_test = zed_pose_test.ekf_sensor_evaluation_test:main',
            'passive_sensor_logger = zed_pose_test.passive_sensor_logger:main',
            'ekf_delay_diagnostic = zed_pose_test.ekf_delay_diagnostic:main',
            'blind_path_follower = zed_pose_test.blind_path_follower:main',
            'compass_dive_test = zed_pose_test.compass_dive_test:main',
            'compass_circle_test = zed_pose_test.compass_circle_test:main',
        ],
    },
)


    

