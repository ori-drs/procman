cmake_minimum_required(VERSION 3.8)
project(procman_ros)

add_compile_options(-std=c++17)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(procman_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)


ament_python_install_package(${PROJECT_NAME})

#######################################
## Build executables and libraries   ##
#######################################

include_directories(include)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(procinfo_cpp procinfo_linux.cpp)
else()
  set(procinfo_cpp procinfo_generic.cpp)
endif()

add_library(procman_ros SHARED src/procman/exec_string_utils.cpp
                           src/procman/${procinfo_cpp}
                           src/procman/procman.cpp)
target_link_libraries(procman_ros util)
ament_target_dependencies(procman_ros rclcpp procman_msgs)

add_executable(deputy
               src/procman_ros/socket_monitor.cpp
               src/procman_ros/procman_deputy.cpp)

target_link_libraries(deputy procman_ros )
ament_target_dependencies(deputy rclcpp)

####################
## Installation   ##
####################

set(targets_list
  deputy
  procman_ros
)

install(
  TARGETS ${targets_list}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install(FILES python/procman-ros-sheriff.glade
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/python)

#install(FILES doc/images/procman_icon.png
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc/images)
        
install(PROGRAMS
        scripts/sheriff
        DESTINATION lib/${PROJECT_NAME})
        
install(PROGRAMS
        python/procman-ros-sheriff.glade
        DESTINATION share/${PROJECT_NAME})
        
install(PROGRAMS
        doc/images/procman_icon.png
        DESTINATION share/${PROJECT_NAME}/doc/images)

ament_package()

