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

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/procman_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/procman_ros_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# Install the python sheriff script to bin so that it does not have to be called
# with rosrun
#set(procman_ros_sheriff ${CMAKE_INSTALL_PREFIX}/procman-ros-sheriff)
#message(${procman_ros_sheriff})
#message(${CMAKE_INSTALL_PREFIX})
#file(WRITE ${procman_ros_sheriff}
#           "#!/usr/bin/env python2\n"
#           "import sys\n"
#           "sys.path.append('${CMAKE_INSTALL_PREFIX}/${py_install_dir}')\n"
#           "import procman_ros.sheriff_gtk.sheriff_gtk\n"
#           "procman_ros.sheriff_gtk.sheriff_gtk.main()\n")
#catkin_install_python(PROGRAMS ${procman_ros_sheriff} DESTINATION bin)

# This is used to find some files required to construct the GUI
#set(build_prefix_py ${CMAKE_CURRENT_BINARY_DIR}/src/procman/build_prefix.py)
#file(WRITE ${build_prefix_py}
#     "BUILD_PREFIX='${CMAKE_INSTALL_PREFIX}'\n")
#install(FILES ${build_prefix_py} DESTINATION ${py_install_dir}/procman)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp")

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_procman_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
