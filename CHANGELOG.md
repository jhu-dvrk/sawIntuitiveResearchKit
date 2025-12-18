Change log
==========

2.4.0 (2025-12-18)
==================

Starting with 2.4.0, we strongly recommend to use ROS2 with Ubuntu 24.04 or higher.

* API changes:
  * New JSON format for:
    * Top configuration level (was console.json, now system.json)
    * PID
    * IO
    * snake_case in all configuration files
    * Si units in all configuration files (radians, meters...)
  * New CRTK Python syntax, all methods return a tuple (data, timestamp). Timestamps set to zero indicate an error.
* Deprecated features:
  * CMake minimum required: 3.16
  * Ubuntu 18.04 support dropped
  * Anything referring to "console", now uses "system"
* New features:
  * Documentation now on https://dvrk.readthedocs.io, major updates and fixes
  * Improved configuration file generator for IO (python script)
  * Improved interface for calibration scripts
  * System configuration file wizard (C++/Qt GUI), save and/or launch a system configuration
  * New top level component "system" (was "console")
    * Support multiple IO components
    * Multiple surgeon consoles
    * Multiple ECMs per patient side
    * Clutch works on MTMs even with no operator present
  * Gravity compensation for S/Si patient side arms (PSM and ECM)
  * All arms have mounting_pitch used for gravity compensation
  * Added support for embedded Python interpreter (aka cisstInteractive or IRE)
  * Updated cisstMultiTask collectors for reliable data collection of events and state data without ROS
  * All components (IO and PID) now have their own ROS bridge library
    * Available in read-only or read-write mode
    * Added simple ROS nodes with IO or IO and PID without arm and system classes
  * Added solution for "fixed" SUJ, setups with PSMs and ECM but no SUJ controller
    * SUJ_fixed class
    * Hand-eye registration script to find PSM_base to camera or ECM_base  
  * New example with bilateral teleoperation, python embedded interpreter...
  * Support for custom endoscopes (DH and GC)
  * Many more instruments supported
  * On SUJ Standard, added weights between primary and secondary pots to support faulty systems
  * PSM teleoperation: flags to enable/disable use of linear/angular velocities
  * ...
* Bug fixes:
  * Fix bug damaging instruments when engaging
  * Ongoing tuning of PID gains
  * Fixed gravity compensation for ECM Standard/Classic
  * Better DH and URDF for PSMs and ECMs, including physical DH for gravity compensation
  * JSON schemas: fixed references, use markdown (was html), added requirements.txt, integrated in readthedocs

2.3.1 (2025-01-17)
==================

* API changes:
  * The dvrk_console_json will close all the relays for the user. You shouldn't need to use `qlacommand -c close-relays` anymore!  This can be overriden in the console JSON config.
  * All arms are re-homed and the encoder preload on the controllers are ignored. It's a bit slower but prevents some issues with potentiometer drift. This can be overriden in the console JSON config.
* Deprecated features:
  * All site specific directories under `share` have been removed. Repositories with the existing files have been created under https://github.com/orgs/dvrk-config/repositories.  Let Anton know if you need access to your site's repository.
* New features:
  * Added more arm_from_ros
  * Added script to reconnect all bluetooth for SUJ Si
* Bug fixes:
  * Added missing include for Ubuntu 24.04
  * Fixed bug when using -I


2.3.0 (2024-08-30)
==================

* API changes:
  * Supports firmware 9
  * Reorganized files so C++ based ROS nodes are included in this repository and can be compiled for both ROS1 and ROS2. `components` and `examples` are now under `core`.  The directory `ros` contains all the ROS dependent applications (i.e. `dvrk_robot`, `dvrk_arms_from_ros`, `dvrk_hrsv_widget`)
  * The old `dvrk-ros` repository, specific to ROS1 has been spit in multiple repositories: `dvrk_python`, `dvrk_video`, `dvrk_model`...  All the new repositories are compatible with ROS1 and ROS2
* Deprecated features:
  * None
* New features:
  * Support for Goovis HMD (https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Goovis)
  * Control Si LEDs
  * Basic gravity compensation for ECM Si (only applies to endoscope weight)
  * PID files use strings for joint types
  * Shared files:
    * Added instruments 400230, 410298, 420230 and 420327
  * New consoles simulated for surgeon console and patient cart (Classic and Si)
* Bug fixes:
  * Faster jaw velocity in teleoperation
  * #203 coupling matrix for `FIXED` instrument type
  * SUJSi can be used without all 5 dESSJ (bluetooth)


2.2.1 (2023-12-29)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * Shared files:
    * Added `tool` mega suturecut needle drivers 400309 and 420309
    * Added `console` for simulated PSM Si and ECM Si
    * Added `io` for MTML on DQLA
  * Console JSON: added configuration option "close-all-relays"
  * Display explanation when powering fails on Si controllers
  * Added CMake option to compile without Bluetooth for SUJ Si in simulated mode
  * `dvrk-remove-logs.py`: added `-f` option to skip confirmation prompt
* Bug fixes:
  * Fixes for Windows compilation
  * Fixed crash on exit
  * Fixed crash on PSM GUI direct control
  * `dvrk-configuration-generator.py`
    * Fixed boards IDs for PSMs and ECMs using DQLA controllers
    * Better default for gripper

2.2.0 (2023-11-21)
==================

* API changes:
  * Configuration files specific to each system/group shouldn't be stored in `share` directory anymore.  We created https://github.com/dvrk-config to host repositories of configuration files.  e.g. for JHU https://github.com/dvrk-config/dvrk_config_jhu
  * More CRTK and snake_case renaming
  * PID config file is now using JSON
* Deprecated features:
  * Ubuntu 16.04 is not supported anymore
  * Ubuntu 18.04 requires using clang as compiler (see documentation)
  * Matlab based configuration generator.  Use `dvrk-config-generator.py` instead
  * `base-frame` for SUJ, wasn't used
  * `rotation` configuration parameter for PSM teleoperation, Use the `base-frame` in console JSON configuration file instead.
  * `mainQtArm` application, should use `sawIntuitiveResearchKitQtConsole` instead
* New features:
  * Support for ROS2 and ROS1 on Ubuntu 20.04 (Galactic/Noetic) and ROS2 on Ubuntu 22.04 (Humble)
  * Support for Si PSMs and ECM
    * IO, PID and kinematics and tool detection working, missing gravity compensation
    * New potentiometer calibration program specific to Si arms
    * Added support for Si SUJ, calibration remains tricky
  * Support for new Classic controllers with a single FPGAv3 (DQLA vs QLA1)
  * Control:
    * PID (see also sawControllers CHANGELOG):
      * Control using actuator space (was using joint space)
      * Removed non-linear gains, fixed deadband transitions
      * Use reference velocity
    * PSM teleoperation:
      * Use MTM pose and twist to control PSM, PSM PID now uses reference actuator positions and velocities
  * SUJ:
    * Refactor of SUJ Classic code
    * Added Fixed SUJ for groups using custom frame to mount their PSMs and ECM
    * Option to set any arm as reference arm.  By default all arm cartesian poses are wrt ECM but this can be changed to any PSM for groups using PSM-held camera
  * New ROS features:
    * Publisher `arm/local/measured_cv`
    * Publisher `arm/gravity_compensation/setpoint_js` provides torques used for gravity compensation
    * Services `arm/forward_kinematics` and `arm/inverse_kinematics`
    * Publisher `arm/crtk_version` (i.e. 1.1.0)
    * Subscribers `arm/free` and `arm/hold`
    * Subscriber `arm/pid_feed_forward/servo_jf` to allow user to add their own feed forward
    * Subscriber `teleop_PSM/following/mtm/body/servo_cf` to add haptic only when teleop is active
    * Add `ral` (ROS Abstraction Layer) for CRTK Python client so Python scripts can be used with either ROS1 or ROS2
  * New sawRobotIO1394 config generator (Python based), also generates a simple arm and console JSON configuration file
  * More instrument definitions
  * Added `dvrk-sd-card-updater.py` to update firmwares for SD cards using in new controllers (Classic with DQLA/FPGAv3 and Si with dRA1/FPGAv3) and Si ESPM
  * Added `dvrk-remove-log.py` to cleanup old cisstLog files
  * CMake:
    * Updated install targets and debian packages generation
    * Works with ROS2/colcon
* Bug fixes:
  * daVinci head sensor: fixed bit Id so all four sensors are used
  * Engage procedures are stopped when adapter/instrument is removed
  * Fixed some transitions for PSM jaw control (`move` to/from `servo`)
  * Fixed emulate adapter/tool

2.1.0 (2021-08-11)
==================

* API changes:
  * Changed instrument identifier from `enum` to `std::string`
  * Instrument name is now following the convention `name:model[version]`.  It used to be `name_model`
* New features:
  * Added new configuration file to list all supported instruments with option to load site specific list: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Custom-Instruments
  * More instruments supported, including new Si Large Needle Driver with gears (version 12+)
  * Instrument depth:
    * Using Classic or S specific depth to determine if instrument should engage
    * Using depth of joint 4 using forward kinematics to restrict motion towards/inside the cannula
  * Updated wiki:
    * Better calibration documentation: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Calibration
    * PSM joint 3 calibration: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Potentiometer-calibration-for-PSM-insertion
    * Classic vs S instruments: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Generations
    * Support for custom instruments: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Custom-Instruments
  * Updated JSON schemas: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Configuration-File-Formats
  * Added example for *cisstRobotPython*
* Bug fixes:
  * Fixed range of motion used to engage sterile adapter and instruments to avoid encoder/pot errors
  * Fixed bug in Matlab config generator for ECM
  * Teleop PSM restarts automatically after an instrument's swap

2.0.1 (2021-05-26)
==================

* API changes:
  * More snake case renaming for C++ code (internal code)
* New features:
  * Console:
    * Added calibration mode (command line option -C) to disable potentiometer checks and saving encoder offsets when calibrating potentiometers
* Bug fixes:
  * Fixed examples of configuration files for data collection using cisstMultiTask state table collectors

2.0.0 (2021-04-08)
==================

Added CODE_OF_CONDUCT

* API changes:
  * Internal *cisst* commands and ROS topics use CRTK naming convention as much as possible
  * Commands without CRTK equivalents have been renamed using `snake_case` (vs previous convention using `CamelCase`) to match ROS/CRTK conventions
  * A conversion script is provided to help porting code to new naming.  The process is not fully automated, porting code to dVRK 2.0 will require some manual updates.  Script can be found on the [cisst](https://github.com/jhu-cisst/cisst/) repository, in `utils/crtk-port`.  The porting script uses translations files (`.dic`).  We provide a few different translation files:
    * `crtk-commands.dict`: *cisst* command names, mostly for C++ code without ROS dependencies
    * `crtk-ros-commands.dict`: ROS topics translation, mostly to port applications using the ROS topics directly
    * `members-saw-intuitive-research-kit.dict`: data members and methods for dVRK C++ code.  This can be used to port C++ code based on dVRK code C++ code
  * Console configuration files shouldn't use `io`, `pid` and `kinematic` for `arms`.  Instead, use `serial` and optionally `system`.  The application will then look for the arm JSON configuration file (e.g. `PSM1-12345.json`)
  * Kinematic config files updated so limits are part of DH parameters, now use SI units!
* Deprecated features:
  * Reduced search path for configuration files, now need to add directory prefix.  For example, to load `"sawRobotIO1394-MTMR-foot-pedals.xml"`, you need to use `"io/sawRobotIO1394-MTMR-foot-pedals.xml"`.  Same applies to `arm`, `kinematic`, `console`.
* New features:
  * General:
    * Supported Linux platforms are Ubuntu/ROS are 16.04/kinetic, 18.04/melodic, 20.04/noetic
    * On Ubuntu, added `rosinstall` files to use with `wstool`.  This is now the preferred way to retrieve the cisst/SAW/dVRK code
    * Added github workflow (https://github.com/jhu-dvrk/dvrk-github-workflow/actions)
    * JSON schemas are used to validate configuration files and generate documentation, see README in `share/schemas`
    * Preliminary support for ethernet/UDP. FireWire remains the preferred interface
    * Preliminary support for MacOS and Windows, no ROS and with ethernet interface only (no FireWire support)
    * sawSocketStreamer and sawOpenIGTLink support using configuration files (README can be found in `share` directory)
    * Dark mode (option `-D`)
  * Console:
    * Added event + ROS to report if teleop is enabled
    * Exposed audio features: set volume, beep and text to speech to ROS
    * Emulate foot pedal events (operator, clutch, camera) using ROS topics
    * Widget:
      * Display list of arms and current state (using color).  Clicking on arm name brings arm widget in focus
      * Display list of tele-op PSM pairs, selectable in GUI.  Clicking on pair name brings pair widget in focus
      * In *Direct mode*, emulate console events (clutch, camera...) using check boxes
      * Added widget for endoscope focus (+/-)
  * Arm:
    * Use CRTK convention for most motion and operating state commands
    * Faster homing by loading preloaded encoders from controllers
    * Added command to get joint configuration (name, type, limits for position, velocity and effort)
    * Added single ratio applied to both velocity and acceleration for `move` commands
    * Added ROS service to query frames along kinematic chain for a given set oof joint values
    * Widget:
      * In *Direct mode*, added widget to move arm using `move_jp` command (joint space) and jaws (for PSM)
      * Added operating state widget with ability to send state commands in *Direct mode*
      * For PSM and MTM, display jaws and gripper joint state
  * ECM:
    * Added gravity compensation
    * Use PID feed forward, better PID tracking, higher maximum velocity for trajectory generation
    * Support multiple endoscope types (SD/HD, straight/up/down)
    * Use closed form IK
    * Widget: added drop down menu to select endoscope type (in manual mode)
  * MTM:
    * In effort mode, apply torque to wrist platform to move away from user's hand.  Force can be scaled using configuration setting `platform-gain`
  * PSM:
    * Added support to change tool type at runtime.  With proper hardware/firmware, get tool type automatically from Dallas Chip
    * Kinematic is now defined using the arm part (first 3 dofs) and tool part (last 3 or 5 dofs) and optional gripper
    * Tool definition files added in `share/tool`
    * Widget:
      * Added drop down menu to select tool type (in manual mode)
      * Added plot to display jaw effort
  * SUJ:
    * Added simulated mode
    * In simulated mode, added command/configuration setting to set joint positions
  * Teleoperation PSM:
    * Better *engage* check to start tele-operation when starting and after clutch
    * Added rate control on PSM jaws, slower when engaging
    * Scale angle to match jaw and gripper ranges
    * Support tele-operation while ECM is moving
    * Widget: display orientation offset between MTM and PSM
  * ROS:
    * Matlab and Python client libraries are now based on CRTK client libraries, see dvrk-ros CHANGELOG.md
    * *cisstMultiTask* dVRK arm using ROS topics, can be used to tele-operate between two computers with ROS as middle-ware
  * sawRobotIO1394:
    * Widget: more closely match sawRobotIO names, shows safety relay status
    * Software will only allow firmware 6 and 7.  7 is recommended
    * Code refactor to reduce number of classes and ease maintenance
    * Moved to format 4 to enforce new features
    * MTMs use two files, one for 7 active joints, one for gripper analog input
  * Utilities:
    * Added `qlacommand` to superseed `qlacloserelays`.  Examples: `qlacommand -c open-relays`, `qlacommand -pudp -c reboot`
    * Added script to reset FireWire kernel modules on PC.  It requires `sudo` privileges: ``sudo `which qlareloadfw.bash` ``
    * Kernel log messages (`dmesg -w`) now display the board id, board type (f for FireWire, e for ethernet) and firmware version
* Bug fixes:
  * Fixed bug re. setting trajectory ratios not always applied
  * Tele-operation for PSM, use alignment mismatch to avoid small jump when engaging

1.7.1 (2019-07-04)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * None
* Bug fixes:
  * Fixed joint velocity abs bug in GC controller: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/pull/117

1.7.0 (2019-04-09)
==================

* API changes:
  * Using c++ 14 features: **Ubuntu 16.04 or higher required**
* Deprecated features:
  * `kinematic` field in JSON console config file has been replaced by `arm` for MTMs to support gravity compensation (see New features)
* New features:
  * Added gravity compensation for the MTM.  Contribution from CUHK. Thank you!
  * Support for multiple teleop components per MTM or PSM (ee issue #97)
    * Using the console, operator can use a "clutch" quick tap to toggle between PSMs.
    * Using ROS topics, programs can select which pairs to activate or de-activate.
  * TeleopPSM: added options to disable jaw on PSM side
  * Teleop Qt widgets: use prmPositionCartesianGet widget
  * Arms: added set ratio velocity and acceleration to control max velocity used in joint tractories
* Bug fixes:
  * Console:
    * Throttle error messages
    * Fixed Coag digital input when operator present is using a different input
    * Fixed valid/timestamp for head sensor events
  * Arms:
    * Fixed valid/timestamp for jaw/gripper joint states
    * Fixed PSM jaw transitions between servo and move modes (see issue #112)
    * Removed reference to deprecated `mtml.json` and `mtmr.json` in configuration files (see issue #106)

1.6.0 (2018-05-16)
==================

* API changes:
* Deprecated features:
  * Fixed reference frames to match ISI convention on real da Vinci systems.  `mtml.json` and `mtmr.json` are now deprecated
  * In arm JSON configuration files, do not use `base-offset`.  Use console JSON configuration `base-frame` instead
  * Teleop PSM/ECM JSON: `rotation` is now moved under `configure-parameter`
  * Standard (i.e. not system specific) IO XML configuration files moved to `share/io` folder.  Console class will search in `share/io` so this should be backward compatible
* New features:
  * Teleoperation ECM:
    * First working implementation, tested on JHU system
    * Use structs for MTMR/L and ECM instead of pointers
  * Teleoperation PSM:
    * Match gripper/jaw angle before starting teleoperation
    * Use structs for MTM and PSM instead of pointers
    * Added option to ignore jaw in PSM if MTM doesn't have a gripper (e.g. Falcon)
  * General:
    * Fixed interval statistics with better load estimation, updated Qt widget
    * Fixed 3D rotation widget, show reference frame and allow to rotate view with mouse
    * Transformation Qt widget now shows moving/reference frame names, valid/invalid flag and timestamp
  * Console:
    * While configuring from JSON file, `exit` if an error is found, this makes it easier to see error messages
    * daVinci head sensor: figured out cabling to connect head sensor from full daVinci system to dVRK controllers, added software support with special component (see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/HeadSensor#davinci-head-sensor)
    * Added audio feedback, emit different beep tones for operator present, master clutch and camera control pedal.  This requires to install sox and espeak (`sudo apt install sox espeak`)
    * Added component to control endoscopic camera from master console foot pedals (requires custom wires, see instructions https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Full-da-Vinci#endoscope-focus-controller)
  * Arm:
    * Added relative position commands in joint and cartesian space
    * Console JSON configuration file, added `base-frame` with static transformation and name of reference frame for each arm
    * For all cartesian velocities/twist, added name of moving frame
    * PSM: added set_effort_jaw command, this can be along with set_effort_joint or set_wrench on PSM
  * SUJ:
    * Better/faster propagation of base frames
    * Added simulation mode, can set joint values using ROS topic
    * Removed "desired" position, all positions are "measured"
    * Joint names are now compatible with WPI ROS/urdf models
  * ROS:
    * Added tf2 support
    * Created multiple ROS bridges, publishers, subscribers, tf2 and cisst/SAW period stats (custom message), much faster processing of subscribers
    * Initial support for crtk topics
  * sawRobotIO1394:
    * New velocity estimation on FPGA with firmware rev 6, improved PID and cartesian impedance controllers
    * Software will not allow firmware different from 4, 5 or 6.  6 is recommended
    * If pot/encoder tolerance distance is set to 0, ignore axis in safety checks
    * Code refactor to reduce number of classes and ease maintenance
* Bug fixes:
  * Fixed data member initialization (valgrind)

1.5.0 (2017-11-07)
==================

* API changes:
  * Console: watchdog default changed to 30 ms.  The default can be overridden in console.json file.
  * Arm: SetRobotControlState has been removed, use SetDesiredState and GetDesiredState/GetCurrentState commands along with DesiredState/CurrentState events
  * Arm: no need to set a new state to switch control mode (position/cartesian position/goal/effort)
  * Arm: joint states size is now based on number of joints used for kinematics.   Gripper (PSM) and jaw (MTM) are reported using a different joint state command/topic
  * Console: digital inputs/outputs not specific to an arm are now defined in a different file.  All sawRobotIO1394-{PSM,ECM,MTM} XML config files need to be updated!
* Deprecated features:
  * Arm: removed hard coded pot tolerance per arm type.  Now uses sawRobotIO XML file to save settings per arm and implementation uses time instead of iteration counter.
* New features:
  * New dVRK logo: https://github.com/jhu-dvrk/dvrk-logo/blob/master/dVRK-blue-05-04-2017-01.png
  * Arm:
    * New current calibration procedure, reduces current offset and undesired torques when request current is zero.  Please regenerate your sawRobotIO configuration files!
    * Use mtsStateMachine to better control initialization and homing, now manages desired vs. current state
    * States for each control mode have been removed, the mode will be automatically set when receiving a new move command
    * Control modes have been added (space: actuator/joint/cartesian/user and mode: position/trajectory/effort/user) instead of states
    * Control mode for user implementations in derived classes can be set using SetControlCallback
    * Use Reflexxes for trajectory generation.  Can now interrupt existing trajectory using current velocity instead of resetting to 0.  Dropped use of robLSPB
    * Cartesian impedance controller has been added to all arms.  Very useful for simple haptic feedback on MTMs.   Some examples are available in dvrk-ros.
    * When using firmware 6, use velocity estimation from controller (much better timestamps and estimation)
    * Uses new standardized mtsMessage for Qt messages and ROS log
  * PSM:
    * Added support for 5mm tools (snake-like tools)
    * For 5mm tools, joint state has 8 joints
    * Added IK with equality constraints to support 5mm tools
    * Added implementation for tool specific torque limits (loaded from psm-xyz.json)
    * Added general purpose socket base protocal to communicate with PSM
    * In psm-_.json, no need to provide all coupling matrices, just need the actuator to position
  * MTM: Homing is now slower to prevent hard hits on roll joint.  Also just looking for lower limit instead of both lower and upper limits
  * Teleop PSM:
    * Added support for alternate master arm (sawForceDimensionSDK)
  * Console:
    * Added support for dynamic loading of external cisst/SAW components (e.g. sawKeyboard, sawForceDimensionSDK, sawSensablePhantom)
    * Support for alternate foot pedals (keyboard) and master arms (Novint Falcon, ForceDimension)
    * Separated configuration files for digital inputs/outputs from arm configuration files (see API changes)
    * Uses new standardized mtsMessage for Qt messages and ROS log
  * Console Qt:
    * Added tag/clear shortcuts in message window
    * Can power all arms without homing, this allows to test safety chain and pot/encoders without PID on
  * SUJ:
    * use state machine compatible with other arms
    * now reports joint values even if the arm is not in state "READY"
    * minor updates for firmware 6
    * use joint state instead or joint position
  * CMake:
    * Everything can now be compiled on Windows when using proper branch of sawRobotIO
  * ROS:
* Bug fixes:
  * Fixed loading joint coupling matrix when the arm is started with a tool in place
  * Fixed jumps at homing and control mode transitions with rewritten PID component



1.4.0 (2016-08-31)
==================

* API changes:
  * PSM teleoperation now uses sawIntuitiveResearchKit class `mtsTeleOperationPSM`, not `mtsTeleOperation` from sawControllers
  * Console: console now handles teleoperation on/off logic as well as PSM and SUJ clutch events
  * Coordinate systems now match ISI conventions, see API wiki page (https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Components-APIs)
  * Added class `mtsStateMachine` used by teleoperation components (NOTE, `mtsTeleOperationECM` is a work in progress, not ready for consumption)
  * ECM: new file format (.rob replaced by .json with DH and tooltip-offset), see file formats wiki page (https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/FileFormats)
* Deprecated features:
  * Arm modes `DVRK_GRAVITY_COMPENSATION` and `DVRK_CLUTCH` will be deprecated in next release, use `DVRK_EFFORT_CARTESIAN` with commands to turn gravity on/off, lock orientation instead (see example in `mtsTeleOperationPSM.cpp`)
  * MTM: removed `SetWrench` method, use `SetWrenchBody` or `SetWrenchSpatial` instead
* New features:
  * Arm:
    * Jacobian body and spatial now stored in state table and accessible via provided interface (array ROS topics)
    * Compute an estimated wrench at tooltip using current feedback converted to joint torques and jacobian body inverse (wrench ROS topic)
    * PSM and ECM don't move when powered on
    * All arms JSON configuration now support `base-offset`
    * Added command `SetBaseFrame` and equivalent ROS topic (`set_base_frame`)
  * PSM:
    * Added JSON configuration file for pro grasp tool
    * Reduced PID gains to increase stability across system (NOTE: make sure you don't use an old PID file)
    * Added command (and ROS topic) to force SetToolPresent (used to test tools without wiring/chip on the back)
  * MTM:
    * Added JSON configuration files for MTM left and right to use stereo display coordinate system
    * Fixed orientation lock, added command to set desired lock orientation
  * Teleop PSM:
    * Uses new `mtsStateMachine` class
    * Checks that PSM and MTM are in proper state
    * Checks that PSM and MTM are properly aligned, issues warning messages in console when not aligned
  * Console:
    * Added option in configuration file to set FireWire protocol, can significantly improve performance on some systems! See `firewire-protocol` in file formats wiki page (https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/FileFormats)
    * Better support for derived teleop components (both PSM and ECM)
    * Manages state transition for teleop PSM/ECM and user events
    * Added search path to avoid absolute directories in console JSON configuration files, now searches in current directory, directory of console file being loaded and then `sawIntuitiveResearchKit/share`.  This allows to find shared files (PID, kinematics) without copying them around or specifying a full/relative path
    * Default period for arm is now 1.0 ms, for IO/PID 0.3 ms (you should remove all `period` from your console JSON config files).
  * Console Qt:
    * Keep console widget/log visible on left/bottom, top/right used for IO, PID, Arm widgets in sub tabs
    * Added widget to set scale for all teleop components
    * Added keyboard shortcuts for basic commands (ctrl+H: home, ctrl+O: power off, ctrl+T: start teleop, ctrl+S: stop teleop)
    * Temporary logo for the dVRK
  * SUJ:
    * Ongoing fixes for dSIB rev 3
    * Uses both sets of potentiometers
    * Checks that both sets of potentiometers agree and issues warning messages
  * CMake: separated components from applications/examples (catkin build 0.4 compatible).   Since directories have been moved around, all your build trees need to be removed/cleaned.  For catkin build tool users, please clean using either `catkin clean -a` (older catkin build tools) or `catkin clean --all` (for all catkin versions supporting `catkin --version`, i.e. 0.4 and above).
  * ROS: `sawIntuitiveResearchKit/share` directory is now a ROS package so one can find configuration files using the package name `dvrk_share`
  * Added CISST_EXPORT. Who knows, we might compile this thing on Windows one day...
* Bug fixes:
  * Data passed from IO to PID to arm doesn't get re-timestamped to preserve original timestamp
  * Arm: fixed computation of cartesian velocities using proper jacobian (twist ROS topic)
  * ECM: fixed inverse kinematics


1.3.0 (2016-01-08)
==================

* API changes:
  * PSM: new file format (.rob replaced by .json with DH, coupling, limits, ...)
  * Added SetSimulated for all arms/PID for kinematic simulation (assumes perfect PID)
  * Console: better support for derived/simulated classes for ECM, PSM and MTM
* Deprecated features:
  * None
* New features:
  * PSM: improved engage procedure for sterile adapter and tool (uses trajectory generator)
  * PSM: increased PID gains significantly now that engage uses trajectories
  * PSM: when homing, do not go to zero position if tool is present
  * PSM: move sterile adapter all the way up when engaged
  * PSM: doesn't engage tool is tip is outside cannula
  * MTM: search for joint limits only first time it is homed (need to restart application to search again)
  * Added procedure to calibrate potentiometers, requires ROS build
  * Added SetWrench body/spatial on MTM/PSM with ROS topics, mode DVRK_EFFORT_CARTESIAN
  * SUJ: added JSON field to set brake current
* Bug fixes:
  * More stable bias encoder from pots, uses multiple samples
  * Matlab config generator: selecting MTML didn't update IO and directions (#52)

1.2.0 (2015-10-18)
==================

* API changes:
  * Fixed digital input bug for state pressed.  Need to regenerate XML IO configuration files!
  * New PID has improved velocity estimation.  Need to use new PID XML files or at least update user files!
* Deprecated features:
  * None
* New features:
  * Console class can now be configured using json file for arms/teleops
  * Single program can be used for different console configurations, including for Qt widgets
  * Preliminary support for SUJ (using alpha version of dSIB)
  * Added support for base frame and commands to retrieve cartesian wrt arm base or base frame (for SUJ)
* Bug fixes:
  * Matlab config generator has been updated to create proper digital input "pressed" values
  * Matlab config generator forces to pick arm type, was generating improper joint directions on MTML
  * Use new velocity estimation, PID gains are now independant of periodicity
  * Fixed bugs in close/open grippers
  * PID configuration files now use joint name matching ROS/urdf files (in dvrk-ros)
  * PSM inverse kinematics now enforces tool can't go back in cannula

1.1.1 (2015-05-15)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * None
* Bug fixes:
  * Removed duplicate method Configure in MTM (base class Arm method is used)
  * PSM: increased pots to encoder tolerance during homing
  * Gripper calibration example: added code to preload encoders to avoid encoder bit overflow

1.1.0 (2015-04-28)
==================

* API changes:
  * SI units (meters) instead of millimeters are now used for all translation joints (ECM and PSM joint 3).  Cartesian positions were always based on SI.  `.rob` files were already meter based.  Users need to pay attention to:
    * PID gains.  We provided new PID configurations files for all arms but if you have old tuned gains for your system, you will need to increase them.
    * ROS joint topics.  If you used the ROS bridges, all joints were reported in millimeters.  MORE IMPORTANTLY, all joint commands for PSM and ECM translations were assumed to be in millimeters.  A move by 1 meant 1 millimeter.  It now means 1 meter!
  * Programmers using distances in the code should use the `cmn_mm` constant (e.g. `double tolerance = 1.0 * cmn_mm;`)
  * When building sawIntuitiveResearchKit, you must have cisst configured with `CISST_USE_SI_UNITS`.
  * Joint 7 on PSM is now referred to as "jaw", e.g. `SetPositionJaw` instead of `angle` or `gripper`.  On MTM, the last joint is still called "gripper"
  * For all derived classes (ECM, MTM and PSM), the enum types used to encode states are now shared across arm types, you will need to replace the ECM, MTM or PSM prefix by DVRK (e.g. `PSM_HOMING_POWERING` is now `DVRK_HOMING_POWERING`).
  * In C++ implementation, events are now defined in different scopes, struct `EventTriggers` is now split into `ClutchEvents`, `GripperEvents` and `MessageEvents`.
* Deprecated features:
  * None
* New features:
  * A new base class has been introduced to share code between arm types: `mtsIntuititiveResearchKitArm`.  The base class implements most of the homing procedure as well as most control modes.  There is still some code duplicated in the derived classes that should be ported to the base class, mostly related to state management.  We are considering the Boost MSM class for future revisions.
  * All arms now feature four control modes:
    * Joint direct: Arm must be ready.  Commands are sent to PID directly.
    * Cartesian direct: Arm must be ready and tool (for ECM and PSM) is past cannula.  Inverse kinematics is computed and new joints goal sent to PID.
    * Joint goal: Same pre-condition as direct joint but the `robLSPB` trajectory generator is called to make sure the robot moves smoothly to goal.  An event is sent when trajectory ends.
    * Cartesian goal: Same pre-condition as direct cartesian but the inverse kinematic is calculated for current and goal positions then trajectory in sent as joint goal.   NOTE: interpolation is performed in joint space.
  * Added commands to retrieve measured vs. desired positions, both for joints and cartesian.
  * Two new events have been added:
    * `RobotState` with payload `std::string` with the `DVRK_xyz` value corresponding to the current state.
    * `GoalReached` with payload `bool` to indicate if the trajectory has been fully executed.
  * The command `SetRobotControlState` can now accept `std::string` matching all possible state values using `DVRK_xyz`
  * The base class for all arms now has event handlers for PID errors (tracking and limits).  If any error occurs, trajectories using LSPB are stopped and an event `GoalReached(false)` is sent to caller.
  * All components now have three events for health status:
    * `Error`, `Warning` and `Status`.  All events have a `std::string` payload.
    * Most components used in the dVRK have been updated to act on error events (change to idle or safe state).  There is no automatic recovery, i.e. the user has to initiate the recovery using GUI and/or ROS interfaces.
    * Errors currently handled:
       * IO level: loss of power, watchdog timeout, firewire errors, pots to encoders discrepencies, encoder bit overflow.
       * PID: tracking errors.  Commands outside joint limits are treated as warning (values are clamped and an event with vector of booleans is sent to user).  When receiving error event, disable PID.
       * Arm: when receiving error event, goes to idle mode.  Need to re-home to use arm.
       * TeleOp: When error is caught or if positions are invalid (`IsValid()`), disables tele-op.
  * Console class is now connected to all IO, PID, arms and teleop components:
    * Catch error/warning and status messages from all components.
    * `Idle` button in Qt Widget allows to turn off all arms in software.
    * `TeleOp` button in Qt Widget allows to turn on all tele-op pairs.
  * Firewire communication now uses write broadcast by default when all FPGA-QLA boards have firmware 4.0 or higher.  For N boards, reduces packets from N reads + N writes to N reads and 1 write.  See `sawRobotIO1394` for details.
  * Matlab XML config file generator now adds a `-foot-pedal` suffix to the generated config file so it's easier to see which controller is configured for the foot pedals.
  * ROS interfaces have been updated to take advantage of new events and commands (see dvrk-ros repository).  Bridges can now be created in a more systematic way using the `dvrk_utilities` with the global functions `dvrk::add_mtm_topics`, ...
  * Tele-op:
    * Now handles error events and doesn't use cartesian positions for master/slave if marked as invalid
    * Updated GUI with messages, labels for master/slave, enable check button based on component state
* Bug fixes:
  * Commands using the inverse kinematics now check the result of `robManipulator::InverseKinematics`.
  * Inverse kinematics for the ECM has been fixed.
  * Examples now build properly with catkin/ROS, i.e. they can be found in ROS path
  * For MTM, when calling inverse kinematics with "current" joint position (initial guess for optimization), set joint 5 to 0 instead of joint 3 to pi/4 to try to move the wrist platform away from the user's hand.  The idea is to use the joint on top of the L bracket to move the L close to 90 deg off gripper.
  * For MTM, inverse kinematics uses closest modulus 2 pi position for last joint.
  * Encoders are now pre-loaded when the IO component starts.
  * IO level now checks encoder bit overflow.

1.0.1 (2015-01-30)
==================

* No change log file, initial release.
