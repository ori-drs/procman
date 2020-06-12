cmake_minimum_required(VERSION 2.8.3)
project(procman_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation
                                        roscpp
                                        rospy)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(FILES  ProcmanCmdDesired.msg
                         ProcmanCmd.msg
                         ProcmanCmdStatus.msg
                         ProcmanDeputyInfo.msg
                         ProcmanDiscovery.msg
                         ProcmanOrders.msg
                         ProcmanOutput.msg)

## Generate added messages and services with any dependencies listed here
generate_messages()

catkin_package(INCLUDE_DIRS include
               LIBRARIES procman_ros
               CATKIN_DEPENDS message_runtime roscpp rospy)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(procinfo_cpp procinfo_linux.cpp)
else()
  set(procinfo_cpp procinfo_generic.cpp)
endif()

add_library(procman_ros SHARED src/procman/exec_string_utils.cpp
                           src/procman/${procinfo_cpp}
                           src/procman/procman.cpp)
target_link_libraries(procman_ros util)

add_executable(procman_ros_deputy
               src/procman_ros/event_loop.cpp
               src/procman_ros/procman_deputy.cpp)

target_link_libraries(procman_ros_deputy procman_ros ${catkin_LIBRARIES})

add_dependencies(procman_ros_deputy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(event_loop_test src/procman_ros/event_loop_test.cpp 
                               src/procman_ros/event_loop.cpp)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp")
