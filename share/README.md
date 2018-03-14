dVRK configuration files
========================

The console class which loads a `console-XYZ.json` file has a search path that include this `share` directory.   You should always use a relative path when using the configuration files from this directory and subdirectories.

# Directories per system

Directories are used to store system specific files.  Directory names start with the institution name
(e.g. jhu for Johns Hopkins, isi for Intuitive Surgical) and should contain the system name (e.g. JHU has two systems,
a research kit: `jhu-dVRK`, and a full da Vinci: `jhu-daVinci`).

We strongly encourage each dVRK site to clone this repository, add your system specific configuration files and then
either issue a pull request or send us (JHU) your configuration files.

Each directory should contain:
  * your IO configuration files, `sawRobotIO1394-xxxxx.xml`, for each arm identified by its
number.  You might also want to store the original `.cal` files provided by Intuitive Surgical since they are needed to re-generate the IO XML files.
  * your console configuration files since these refer to your system specific IO configuration files

# Shared files

## IOs

There are some IO files shared accross systems/sites.  These are used to define inputs/outputs used to communicate with the daVinci foot pedals, head sensor, camera focus controllers, ...

These files are stored in the subdirectory `io`.  The console class will search for files in this directory, there's no need to specify the full path to the standard IO files. 

## PID

The PID files, `sawControllersPID-xxx.xml`, should work fine on most systems.  There are 4 files provided, `ECM`, `PSM`, `MTML` and `MTMR`.  We maintain two MTM configuration files simply because the joint limits are different.  Everything else should be the same.  If you need to edit one of the MTM files, make sure you keep the other in sync.

## Kinematics/Arm configuration

The kinematic parameters (DH and dynamic model) are store in two different types of files: `.rob` and `.json`.  Historically we started with the `.rob` files but as we needed to add more settings to configure each arm we moved to JSON.
 * PSM:
   * `dvpsm.rob`: not used anymore
   * `psm-<tool-type>.json`: includes DH parameters and other info specific to each tool.  **Important:** some parameters (e.g. DH of first 3 dofs) are shared across all `psm-xxx.json` files.  If you find an issue in one, fix them all.
 * MTM:
   * `dvmtm.rob`: not used anymore
   * `mtm.json`, `mtml.json` and `mtmr.json`: DH and dynamic parameters for MTMs.  The `mtml` and `mtmr` version include a base transformation to match the ISI API convention, i.e. use the stereo display as base frame.  **Important:** most parameters are shared across all `mtm{,l,r}.json` files.  If you find an issue in one, fix them all.

## Console

Most configuration files are system specific but if you need to simulate a single arm (kinematic simulation only so far), you can use one of the `console-XXX_KIN_SIMULATED.json` file.  This should likely be used with the `dvrk-ros`/`dvrk_robot` ROS package to start RViz with the proper CAD models.

## ROS IO

These files can be used to configure the `dvrk_robot`/`dvrk_console_json` ROS application to publish the IO data of one or more arms.  See the `dvrk-ros`/`dvrk_robot` package.

## Obsolete files

* `dvmtm.rob` and `dvpsm.rob`.  Use the JSON files instead.  The `.rob` files can still be used with `robManipulator` in C++/Python if needed.
* `two-arms.json`.  These files were used with the old application teleop JSON.  This application has been replaced by the Qt console JSON and for ROS `dvrk_console_json`.  The "console" applications are far more convenient and configurable.  Search for existing `console-xyz.json` in this directory and subdirectory to find different examples of use.

