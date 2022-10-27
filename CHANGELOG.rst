^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package procman_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



0.1.7 (2022-10-27)
------------------
* fix conversion from ms to s
* fix not exiting correctly due to unassigned variable when not starting roscore
* roscore is correctly terminated on window close, fixes `#30 <https://github.com/ori-drs/procman_ros/issues/30>`_
* Contributors: David Wisth, Michal Staniaszek

0.1.6 (2022-05-20)
------------------
* add wait until script is finished
* Contributors: David Wisth

0.1.5 (2022-02-22)
------------------
* Fix console output not updating correctly
* Contributors: Michal Staniaszek

0.1.4 (2022-02-07)
------------------
* Revert "orders/info subscribers no longer use infinite queue size", this caused deputies to often lose communication for longer periods of time
* Contributors: Michal Staniaszek

0.1.3 (2022-02-03)
------------------
* belatedly set version to 0.1.2 and add changelog up to that point based on semi-arbitrary breakpoints
* orders/info subscribers no longer use infinite queue size
  I think this could cause issues with the deputy updates and maybe with the stale
  orders issue. After disconnecting from the roscore wifi, deputy treeview shows
  red on the deputies as expected. Previously, reconnecting seemed to take a long
  time. With this I think the info messages come through quicker
* when using observer mode no longer need to specify a config file, fixes `#33 <https://github.com/ori-drs/procman_ros/issues/33>`_
* make sheriff node anonymous so that observers can be run, attempts to fix `#32 <https://github.com/ori-drs/procman_ros/issues/32>`_
* ignore config when starting in observer mode, rather than exiting
* Contributors: Michal Staniaszek

0.1.2 (2021-05-21)
------------------
* build with C++14, Correct usage of ros timestamp tonsec to match with microsecond timestamp_now, use digit separators, timestamp_now uses chrono
  This should fix the problem of getting complains in the sheriff of stale orders messages, this was caused by the ros timestamp returning nsecs but not correctly converting it to microsecs in the stale orders check
  Digit separators for better readability, require c++14
* make the procman icon available when the package is installed, to make debian function too
* add very simple icon
* add note about environment variables
* update readme to be clearer about names of deputies and what lone-ranger does
* fix configs having to be loaded twice before anything was actually loaded
  fixes `#21 <https://github.com/ori-drs/procman_ros/issues/21>`_
  This issue was caused by an extraneous else in the load config which would skip actually loading the requested config if there were any commands already loaded. Just removing the else is not enough because one of the preconditions later down the line is that there are no commands loaded. Because the remove_command function is only a scheduled removal, I added a short wait in after sending those commands. This wait may not be reliable if the network is slow, but any alternative solutions like checking if commands are scheduled for removal rather than just checking if they have already been removed is more difficult.
* remove some final remaining bits of lcm stuff in the code. Apparently never encountered in standard operation so should be fine...
* fix scripts not being able to call other scripts
  This issue is caused by the get_script function trying to acquire a lock which is already acquired when the get_next_action function of ScriptExecutionContext tries to call get_script. This appears to be a new issue on focal/python3, but from what I can see there was no change to threading.Lock.
  It is be possible to fix this by using threading.RLock instead but that may break other things.
* install the glade file to ensure debian package has it
* add install in cmakelists for debian packaging
* Update license references, add myself as maintainer update package xml
* Contributors: Michal Staniaszek

0.1.1 (2021-03-09)
------------------
* change topic names to be more obviously connected to procman
* update readme to reflect use of pygobject and gtk+3
* fix deputy view right click popup and mnemonic
* copying text from console with ctrl-c no longer crashes
* script editing corrected by fixing unescaped brace and missing parameter
* new script dialog correctly opens
* add/edit command dialog initialisation fixed
* update preferences dialog to use proper initialisation of objects
* fix unescaped braces in format strings when saving configs
* initialise menuitems for right click menu on command with new_with_mnemonic, fixes the underscore displaying in the menu text
  Seems that the mnemonics given are actually overridden by the ones provided in the glade file?
* correctly initialise tag, properly displays colours in console
* fix 2button_press reference and popup arguments, can start/stop things with right click
* remove extra calls to reload config on command deletion when loading a config
  This caused an assertion error for each command which was in the previous configuration as the initial load config deleted the attribute which was being checked
  Appears not to have any negative effect on removal of commands
* remove last remaining uses of deprecated get_data
* fix functions passed to TreeViewColumn.set_cell_data_func to take new *data positional arg
* fix background/text colour parsing in console
* fix missing attribute crashing script loading, with open and correct binary read/write to pickle
* use correct functions to access adjustment, remove some deprecated get_data calls
* minimal non-crashing startup, but still has lots of errors
  updated a few things to use gtk3+ syntax/methods
* change file() function to open(), as required in python3
* remove some lcm references
* Apply pygi-convert on python files
  https://gitlab.gnome.org/GNOME/pygobject/raw/master/tools/pygi-convert.sh
  Required because pygtk is no longer supported on focal
  https://askubuntu.com/questions/97023/why-cant-i-import-pygtk-with-python-3-2-from-pydev
* do not start own roscore by default
* better handling of config file errors, properly exits program
* more informative add command error messages, no longer crash when there is one
* fix `#16 <https://github.com/ori-drs/procman_ros/issues/16>`_ bad indent causing gui not to exit on interrupt
* Increase queue sizes to prevent messages being dropped
  Small queues may cause command status to be unknown for arbitrarily long periods of time depending on luck of when messages are receives on pm_orders topic
  fixes `#14 <https://github.com/ori-drs/procman_ros/issues/14>`_
* add some super basic debug info/config
* cpu load display on deputy set to 4 decimal places
* warn and anonymise node when deputy name is not a valid ros name
* fix unused result on system call
* only ros::init after receiving deputy id
  Use the deputy id in the ros node name to ensure that multiple deputies don't kick each other off
* change license to BSD 3 clause
* Contributors: Albert Huang, Michal Staniaszek

0.1.0 (2020-07-21)
------------------

* roscore no longer persists by default after sheriff/deputy exit
* Merge pull request `#12 <https://github.com/ori-drs/procman_ros/issues/12>`_ from ori-drs/fix-mem-cpu-usage
  Fix incorrect display of memory/cpu usage for commands which spawn children
* add function to aggregate memory and cpu for parent+child processes and use it instead of only looking at the parent
  Also format procinfo_linux
* better variable names, no longer use array to store process/system info
* wait until the core is available in parent before continuing
* deputy can now start a roscore if one does not exist, python roscore start variable named to be less confusing
* make observer and lone ranger mutually exclusive
* sheriff now starts roscore if one does not exist yet
* use host instead of deputy as the key for deputy names, to keep compatibility with existing config files
* use idle add in procman output callback, this should fix segfaults as described in `#3 <https://github.com/ori-drs/procman_ros/issues/3>`_
* stop using ros timers, they may be causing threading issues
* remove timers from event loop but retain socket monitoring
* Merge branch 'master' into remove-eventloop
* deputy timers now ros walltimers, try moving some stuff out of eventloop
* update readme with rosrun syntax
* partial solution for `#4 <https://github.com/ori-drs/procman_ros/issues/4>`_, but still using time functions from both ros and system
* fix `#7 <https://github.com/ori-drs/procman_ros/issues/7>`_, event loop quit now calls ros shutdown, remove duplicate headers
* move deputy time initialisation into constructor body to avoid issues when deputy starts before roscore
* Fixes `#5 <https://github.com/ori-drs/procman_ros/issues/5>`_ where starting deputy before roscore can cause a segfault
* shorten procman_ros_sheriff and deputy to just sheriff and deputy
* fix script output not appearing in text box
* add publishers and subscribers, fix run function to process ros messages
* procman orders message is correctly sent
* deputy publishes info about itself and sheriff receives it
* make unused lambda args explicit, use ros timers instead of gobject in some places
* argparse in sheriff_cli
* manual conversion of % formatting to .format
* apply black formatting
* apply pyupgrade to change formatting strings and other older python stuff
* fix indexing into argparse namespace
* apply 2to3 script to update print and other statements
* use argparse instead of getopt
* non-crashing system which can be run with rosrun and no need for install command
* cmakelists installs some more files into the correct place, renamed package to procman_ros
  Removed some lcm objects in the sheriff and replace a few subscribers with ros ones
* Python setup, import ros message names
  Add some of the required files for ros python setup, not entirely complete, still need to install the script to usr/local/bin or elsewhere to make it accessible
  ROS message names are imported and the lcm messages no longer are, and replaced references to lcm messages, but didn't change anything in terms of processing so everything still doesn't work
* Contributors: Michal Staniaszek

0.0.1 (2020-05-04)
------------------
* minimal compiling version of all c++
  LCM stuff that hasn't been ported yet is commented with a //TODO
* initial porting from LCM. Procman library and message generation compile
* updated readme, gitignore
* c++11
* c++11
* don't restart commands when loading from config
* add LICENSE file
* bugfix
* fix sheriff spinning on CPU in observer mode
* bugfix - socket handling
* Adding easy text box copying via copy-paste.
* procman-sheriff script don't set PYTHONPATH
* worder thread send order bugfix
* env var parsing bugfix
* deputy stopcommand bugfix
* fix parallel build error in lcmtypes.cmake
* split deputy into libprocman and deputy
* cleanups, bugfixes
* add doxypypy.py
* more refactoring
* some refactoring
* rename some Python API methods
* remove SheriffCommandSpec
* bugfixes
* add initializer arguments to SheriffCommandSpec
* process stdout/stderr nagling
* bugfixes
* Linux bugfixes
  also:
  - sheriff display memory RSS instead of VSIZE
* deputy switch to custom event loop
* stop using g_shell_parse_argv()
* minor refactoring create exec_string_utils
* load config remove all commands first
* remove move_cmd_to_deputy
* nickname -> command_id
* Guard SheriffDeputyCommand, SheriffDeputy w/lock.
* protect SheriffDeputy attributes with lock
* lcmtypes_build_c minor cleanup
* cmake pass build include path to lcmgen function
* purge options from message types
* deputy name/host -> deputy_id
* cleanup. purge signal_slot.py
* Sheriff switch from signals to SheriffListener
* purge sheriff_id, use command_id as unique id.
* refactor. move scripting into sheriff_script.py
* procman sheriff start switch to multithreading
* cleanup
* src/deputy -> deputy
* cleanups
* bugfixes
* more cleanup
* some cleanups
* procman_deputy switch to Qt5, stop using glib
* VariableExpander
* more c++ conversions
* Procman struct -> class
* more c++ conversions
* remove DeputyCommand::sheriff_id
* procman_cmd_t -> ProcmanCommand
* more c++ conversions
* remove procman_cmd_t::user
* c struct -> C++ struct
* GList -> std::vector
* start using std::map instead of GHashTable
* convert some glib types to stl
* procinfo split to procinfo\_{generic,linux}
  also:
  - start replace GArray with std::vector
  - rename procman_cmd_t::cmd_id -> sheriff_id
  - rename procman_cmd_t::cmd_name -> cmd_id
* deputy add namespace procman
* procman deputy begin conversion to c++
* rename lcm types
* move lcmtypes into package procman_lcm
* cleanup build system
* remove bot\_ prefix
* remove legacy messages
* import bot2-procman
* Contributors: Albert Huang, Benjamin Brown, Marco Camurri, Pedro Vaz Teixeira
