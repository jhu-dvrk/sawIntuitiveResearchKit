Change log
==========

2.2.1 (2023-12-29)
==================

* API changes:
  * Add `deinterlace` for gscam launch for DeckLink cards
* Deprecated features:
  * None
* New features:
  * Added `-s` to publish raw voltages for SUJs
* Bug fixes:
  * None

2.2.0 (2023-11-21)
==================

* API changes:
  * :warning: All topics `measured_cp` and `setpoint_cp` use `PoseStamped` instead of `TransformStamped`
  * `video.md` and video launch files moved to newly created `dvrk_video` package (still part of this repository)
* New features:
  * Support for Si PSMs, ECM and SUJ
  * All video related files (launch and md) moved to ROS package `dvrk_video` to avoid build dependencies on the full dVRK stack
  * Added `vcs` files (replacing `wstool`) for dVRK 2.1, 2.2 and devel
  * Added command line options to expose more IO and PID topics
  * `dvrk_console_json` can locate `ros-io-<arm>.json` files using internal path, no need to specify the full path
  * All Python examples updated to use newly introduced `crtk.ral` (ROS Abstraction Layer) and `crtk.check_connections`
  * Added `dvrk_reset_teleoperation.py` to reposition MTMs and PSMs to better position between teleoperation tasks, very useful for user studies!
  * `dvrk_bag_replay.py`: can now replay using `setpoint_jp` or `setpoint_cp`, fixed timing, doesn't use full `dvrk.psm` but creates a light class using `crtk.utils`, record and send joint velocities for better trajectory following
* Bug fixes:
  * Many

2.1.0 (2021-08-10)
==================

* API changes:
  * Potentiometer ROS topics use joint state messages
  * Instrument name is now following the convention `name:model[version]`.  It used to be `name_model`
* New features:
  * Added Python example `dvrk_bag_replay` to replay trajectory from a ROS bag
  * Added Matlab and Python examples for new CRTK feature `while move_handle.is_busy()`
* Bug fixes:
  * Python 3

2.0.1 (2021-05-26)
==================

* API changes:
  * None
* New features:
  * dvrk_console_json:
    * Added calibration mode (command line option -C) to disable potentiometer checks and saving encoder offsets when calibrating potentiometers
    * Added command line option to use cisstMultiTask state table collectors (-c)
* Bug fixes:
  * None

2.0.0 (2021-04-08)
==================

* API changes:
  * All ROS topics have change to follow the CRTK naming convention, see CHANGELOG for sawIntuitiveResearchKit
  * ROS namespaces don't use the prefix `dvrk`.  Use the standard ROS option `__ns:=` if you need a specific namespace
* Deprecated features:
  * None
* New features:
  * Added `.rosinstall` file
  * Using Matlab and Python client based on CRTK clients - APIs are now consistent across languages
  * Matlab python now used `+dvrk` to create dVRK package (see https://github.com/collaborative-robotics/crtk_matlab_client)
  * ROS bridge uses *cisst-ros* CRTK bridge for most ROS topics
  * Added `dvrk_hsrv_widget`, a simple widget that can be displayed in surgeon's console to show dVRK status
  * Added experimental script to calibrate PSM joint 3 (inserttion joint): `dvrk_calibrate_potentiometer_psm.py`
  * Added `dvrk_arm_from_ros` which allows to create a dVRK console with a remote arm over ROS.  This can be used for tele-operation across network
  * More topics exposed, endoscope focus, volume, beep, text to speech, forward kinematic...  See full API: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/API-2.x
* Bug fixes:
  * Better ROS shutdown for C++ applications
  * Support ctrl-c for Python code
  * Plenty

1.7.0 (2019-04-09)
==================

* API changes:
  * ROS namespaces are now relative, `dvrk_console_json` uses the namespace `dvrk/`, not `/dvrk/`.
* Deprecated features:
  * Old deprecated examples have been removed
* New features:
  * Support Python3
  * Log file (`cisstLog.txt`) is now timestamped
  * ROS topics to:
    * Set velocity/acceleration ratio for trajectory generation
    * Set and check active teleop PSM components
* Bug fixes:
  * Fixed issue with `coag` being hidden if used for operator present
  * Launch files use `xacro`, not the deprecated `xacro.py`

1.6.0 (2018-05-16)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * dvrk_robot:
    * Add tf2 support
    * Multiple threads used, one for publishers, one for tf2 and one for subscribers.  This reduces latency on all subscribers
    * Publishes interval statistics for IO component as well as ROS bridges
    * SUJ joint state subscriber when in simulation mode
    * Added psm `set_effort_jaw` subscriber
  * Python:
    * Added PSM effort test program
    * Added cartesian impedance MTM test program
  * dVRK logo available in STL format for RViz and Gazebo
  * video.md: added some documentation and launch files for DeckLink frame grabbers
* Bug fixes:
  * Fixed missing latch on event publishers
  * `dvrk_console_json` should now quit on ctrl+c


1.5.0 (2017-11-07)
==================

* API changes:
  * dvrk_robot:
    * Update to match new arm state machine
    * Joint commands are now using joints for kinematics only.  On PSM, jaw is not the last joint anymore, it has its separate topics
    * PSM jaw and MTM gripper now use joint state to report position/velocity/effort
    * Better support for console/footpedals events
  * Python:
    * Added blocking/non blocking flag for move commands using trajectory generation
  * Matlab:
    * Added flag to send direct move commands (vs. trajectory goals)
    * Removed all callbacks since these tended to block the interpreter, now use getters to get latest value received.  Getters return timestamp as well.
    * Added ecm.m, console.m and teleop_psm.m
    * Code factorization for all conversion methods
  * Models/URDF:
    * PSM now subscribes to joint state for jaw to control last joint
    * Added crude models for 5mm tools
    * Fixed some xacro warnings
* Deprecated features:
  * None
* New features:
  * dvrk_robot:
    * Teleop now has an event to track if PSM is folowing master
    * SUJ publishes joint state
    * Publishes io interval statistics
    * Added topics for cartesian impedance controller + examples
    * Support for some topics when using generic MTMs (Falcon/ForceDimension)
  * Python:
    * Added test/example programs in dvrk_python/scripts
    * Added subscribers for jacobians, set effort joint
  * Matlab:
    * Added test/example programs in dvrk_matlab/test
    * Added subscribers for jacobians, set effort joint
* Bug fixes:
  * dvrk_calibrate_potentiometers now uses arm.py


1.4.0 (2016-08-31)
==================

* API changes:
  * Python:
    * API uses numpy arrays for joints and PyKDL for 3D commands, no more Python arrays/lists
    * Import as a package using `import dvrk`
    * Added base class arm.py with common features as well as mtm.py, ecm.py, psm.py for arm specific topics
    * Added wrappers console.py, suj.py and psm-teleop.py for corresponding ROS topics
  * Matlab:
    * API more closely matches Python interface
    * Use with CAUTION, it works fine in interactive mode but in a script, Matlab tends to never call the ROS spin function and the callbacks are failing.  See more comments in the dvrk_matlab readme.md
* Deprecated features:
  * Message types have changed to take adavantage of Stamped data type (mostly for timestamps).  See compatibility mode in new features.
  * dvrk_kinematics, dvrk_teleop and rqt_dvrk are now deprecated, still available in directory `deprecated`
* New features:
  * dvrk_robot:
    * Added many topics to match more closely the C++ available commands (see dVRK API wiki page)
    * Added compatibility mode to maintain old topic names and message types.  For example, `dvrk_console_json` has the option `--compatibility <value> : compatibility mode, e.g. "v1_3_0", "v1_4_0" (optional)`
* Bug fixes:
  * Better timestamps (cisst-ros new feature)

1.3.0 (2016-01-08)
==================

* API changes:
  * Python: now import as a package using `import dvrk_python.robot`
* Deprecated features:
  * None
* New features:
  * Potentiometer calibration script: `dvrk_robot/scripts/dvrk_calibrate_potentiometers.py`
  * dvrk_robot: added low level IO data collection for pots/joints/actuators (see potentiometer calibration wiki)
  * dvrk_robot: added set_wrench topics
  * dvrk_robot: console supports kinematic simulation with RViz (optional)
* Bug fixes:
  * None

1.1.1 (2015-10-18)
==================

* Change log file created
* API changes:
  * Removed joint_publisher python code, now use topics directly from dVRK C++ stack (so much cleaner!)
  * Config file name, urdf and rviz, use dVRK C++ arm naming convention, i.e. PSM1, not psm_one, ...
  * Updated al urdf code to handle arm name as parameter (e.g. PSM1, PSM2)
* Deprecated features:
  * None
* New features:
  * dvrk_python/src/robot.py can be used a a simple Python API hiding all the ROS topics/data types
  * dvrk_matlab/robot.m can be used a a simple Matlab API hiding all the ROS topics/data types
* Bug fixes:
  * fixed urdf for PSM jaw angle
  * ...
