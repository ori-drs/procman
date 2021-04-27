# Procman ROS
Porting of the orignal [Procman](https://github.com/ashuang/procman) package to ROS.

# Procman

Procman is a tool for managing many processes distributed over one or more
computers. There are several ways to use procman:

## Sheriff / Deputy GUI mode

In this mode, every workstation runs a *deputy* process:

```
rosrun procman_ros deputy
```

If you like, you can specify a name for the deputy with `-i name`. By default, the deputy will use the hostname of the machine it is run on.

One workstation runs a *sheriff* process, which provides a GUI to command and
communicate with the deputies:

```
rosrun procman_ros sheriff procman_config.pmd
```

Using the GUI, you can:
-  create/edit/remove processes
-  start/stop/restart processes
-  aggregate processes together into logical groups (e.g., "Planning")
-  view the console output of each process
-  save and load process configuration files
-  view process statistics (memory, CPU usage)

If you want to start processes on the machine running the sheriff process, the sheriff can start its own deputy. To operate in this *lone ranger*
mode, run

```
rosrun procman_ros sheriff --lone-ranger
```

Alternatively, you can start a deputy independently as above. Note that when the sheriff starts a deputy its name will be `localhost`.

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
* PyGObject/GTK+3

Currently only tested on GNU/Linux.  Some stuff will definitely only work on
Linux (e.g., the process memory, CPU statistics).

### Debugging

To view debug output, start deputies with

```
ROSCONSOLE_CONFIG_FILE=`rospack find procman_ros`/config/rosconsole/debug.config rosrun procman_ros deputy -i localhost -v
```

### Documentation

Documentation is built with Doxygen.

```
cd doc
doxygen
```
## Credits
- [Original program](https://github.com/ashuang/procman) by [Albert Huang](https://github.com/ashuang). 
- Catkinization and porting from LCM to ROS by [Marco Camurri](https://github.com/mcamurri) and [Michal Staniaszek](https://github.com/heuristicus)
- Porting to PyGObject/GTK+3 by [Michal Staniaszek](https://github.com/heuristicus)

## License
This software is released under the BSD-3 Software License. See the LICENSE for more details.
