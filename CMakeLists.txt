cmake_minimum_required(VERSION 3.8)
project(procman_ros)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
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
  DEPENDENCIES std_msgs)


# ## Uncomment this if the package has a setup.py. This macro ensures
# ## modules and global scripts declared therein get installed
# ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
# 
# 
# catkin_package(INCLUDE_DIRS include
#                LIBRARIES procman_ros
#                CATKIN_DEPENDS message_runtime roscpp rospy)
# 
# include_directories(include
#                     ${catkin_INCLUDE_DIRS})
# 
# if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
#   set(procinfo_cpp procinfo_linux.cpp)
# else()
#   set(procinfo_cpp procinfo_generic.cpp)
# endif()
# 
# add_library(procman_ros SHARED src/procman/exec_string_utils.cpp
#                            src/procman/${procinfo_cpp}
#                            src/procman/procman.cpp)
# target_link_libraries(procman_ros util)
# 
# add_executable(deputy
#                src/procman_ros/socket_monitor.cpp
#                src/procman_ros/procman_deputy.cpp)
# 
# target_link_libraries(deputy procman_ros ${catkin_LIBRARIES})
# 
# add_dependencies(deputy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 
# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.hpp")
# 
# install(TARGETS deputy procman_ros
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
# 
# install(FILES python/procman-ros-sheriff.glade
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/python)
# 
# install(FILES doc/images/procman_icon.png
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc/images)
# 
# catkin_install_python(PROGRAMS scripts/sheriff
#                       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

ament_package()