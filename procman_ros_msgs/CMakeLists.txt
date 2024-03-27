cmake_minimum_required(VERSION 3.8)
project(procman_ros_msgs)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ProcmanCmdDesired.msg"
  "msg/ProcmanCmd.msg"
  "msg/ProcmanCmdStatus.msg"
  "msg/ProcmanDeputyInfo.msg"
  "msg/ProcmanDiscovery.msg"
  "msg/ProcmanOrders.msg"
  "msg/ProcmanOutput.msg"
  DEPENDENCIES std_msgs
)

ament_package()
