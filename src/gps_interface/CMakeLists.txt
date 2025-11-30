cmake_minimum_required(VERSION 3.8)
project(gps_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# List ALL your .msg files here (from Labs 1/2/3)
# Adjust this list to match what you actually have
set(MSG_FILES
  msg/Customgps.msg
  msg/Customrtk.msg
  msg/Vectornav.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
