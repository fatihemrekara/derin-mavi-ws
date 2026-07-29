cmake_minimum_required(VERSION 3.16)
project(rovio_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/SrvResetToPose.srv"
        "msg/Health.msg"
        DEPENDENCIES geometry_msgs
        DEPENDENCIES std_msgs
)
ament_package()