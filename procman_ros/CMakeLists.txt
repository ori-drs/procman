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
find_package(ament_cmake_python REQUIRED)
find_package(procman_ros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
 set(procinfo_cpp procinfo_linux.cpp)
else()
 set(procinfo_cpp procinfo_generic.cpp)
endif()

add_library(procman_ros SHARED src/procman/exec_string_utils.cpp
                               src/procman/${procinfo_cpp}
                               src/procman/procman.cpp)
ament_target_dependencies(procman_ros procman_ros_msgs)
target_link_libraries(procman_ros util)

add_executable(deputy
               src/procman_ros/socket_monitor.cpp
               src/procman_ros/procman_deputy.cpp)
ament_target_dependencies(deputy procman_ros_msgs rclcpp)
target_link_libraries(deputy procman_ros)

# ## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.hpp")

install(TARGETS
  procman_ros
  DESTINATION lib
)

install(TARGETS deputy 
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/sheriff
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES procman_ros/procman-ros-sheriff.glade
        DESTINATION share/procman_ros)
 
install(FILES doc/images/procman_icon.png
        DESTINATION share/${PROJECT_NAME}/doc/images)

ament_export_include_directories(include)
# ament_export_libraries(procman_ros)
ament_export_dependencies(rclcpp rclpy std_msgs)
ament_package()
