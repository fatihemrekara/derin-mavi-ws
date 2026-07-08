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
            'gps_to_local  = zed_pose_test.gps_to_local_node:main',
            'route_planner = zed_pose_test.route_planner_node:main',
            'zed_pose_logger = zed_pose_test.zed_pose_logger_ros2:main',
            'path_follower = zed_pose_test.path_follower_node:main',
            'fake_robot_sim = zed_pose_test.fake_robot_sim:main',
        ],
    },
)


    

