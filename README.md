# Procman ROS
Porting of the orignal [Procman](https://github.com/ashuang/procman) package to ROS.

# Procman

Procman is a tool for managing many processes distributed over one or more
computers. There are several ways to use procman:

## Sheriff / Deputy GUI mode

In this mode, every workstation runs a "deputy" process:

```
rosrun procman_ros deputy
```

One workstation runs a "sheriff" process, which provides a GUI to command and
communicate with the deputies:

```
rosrun procman_ros sheriff
```

Using the GUI, you can:
-  create/edit/remove processes
-  start/stop/restart processes
-  aggregate processes together into logical groups (e.g., "Planning")
-  view the console output of each process
-  save and load process configuration files
-  view process statistics (memory, CPU usage)

For the special case where you only want to run processes on the local
computer, the sheriff can act as its own deputy.  To operate in lone ranger
mode, run

```
rosrun procman_ros sheriff --lone-ranger
```

## C++ API

Procman also provides a C++ API for spawning and managing child processes,
comparable to the Python subprocess module.

## Build Instructions
The package has been catkinized. To compile it, just run:
```
catkin build procman_ros
```
in your workspace

### Dependencies
* rocpp
* rospy
* Python
* PyGTK  (procman-sheriff is written in Python with PyGTK)

Currently only tested on GNU/Linux.  Some stuff will definitely only work on
Linux (e.g., the process memory, CPU statistics).

### Documentation

Documentation is built with Doxygen.

```
cd doc
doxygen
```
## Credits
- [Original program](https://github.com/ashuang/procman) by [Albert Huang](https://github.com/ashuang). 
- Catkinization and porting from LCM to ROS by Marco Camurri and Michal Staniaszek

## License
This software is released under the LGPLv2.1 Software License. See the LICENSE for more details.
