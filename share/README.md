dVRK configuration files
========================

The console class which loads a `console-XYZ.json` file has a search
path that include this `share` directory.  You should always use a
relative path when using the configuration files from this directory
and subdirectories.

# Directories per system

Configuration that are specific to each site should be saved using the
site's repository under https://github.com/dvrk-config

# Shared files

## IOs

There are some IO files shared accross systems/sites.  These are used
to define inputs/outputs used to communicate with the daVinci foot
pedals, head sensor, camera focus controllers, ...

These files are stored in the subdirectory `io`.  The console class
will search for files relative to the "share" directory, there's no
need to specify the full path to the standard IO files but you will
need the `io/` prefix.  For example,
`io/sawRobotIO1394-MTMR-foot-pedals.xml`

## PID

The PID files, `sawControllersPID-xxx.xml`, should work fine on most
systems.  There are 4 files provided, `ECM`, `PSM`, `MTML` and `MTMR`.
We maintain two MTM configuration files simply because the joint
limits are different.  Everything else should be the same.  If you
need to edit one of the MTM files, make sure you keep the other in
sync.  All files are in the `pid` directory and when you include them,
use the `pid/` prefix.

## Kinematic configuration

The kinematic parameters (DH and dynamic model) are store in two different types of files: `.rob` and `.json`.  Historically we started with the `.rob` files but as we needed to add more settings to configure each arm we moved to JSON.
 * PSM:
   * `dvpsm.rob`: not used anymore
   * `psm.json`: includes DH parameters for the first 3 joints.   Since rev. 1.8, tools have their own description files in folder `tool`.  You need to create an `arm` configuration file which describes how the tools are determined (automatic, manual, fixed).
 * MTM:
   * `dvmtm.rob`: not used anymore
   * `mtm.json`: DH and dynamic parameters for MTMs.
   * `mtml.json` and `mtmr.json`: these files are **deprecated**, they include a base transformation to match the ISI API convention, i.e. use the stereo display as base frame.  As of **version 1.6** it is recommended to use the `base-frame` option in the console.json file to specify the base frame.
 * ECM:
   * `ecm.json`: includes all DH parameters except last transformation for up, down or straight endoscope.  You can specify the endoscope type in the ECM arm configuration file

All shared kinematic files are in the directory `kinematic`.

## Tool

PSM tool configuration files are in the `tool` directory.

## Arm configuration

Each arm configuration is specific to your hardware, as identified by
serial number.  Some arm configuration are provided for hardware
independent applications (e.g. kinematic simulation).  You can find
the arm configuration files used for simulation in the directory
`arm`.

## Console

Most configuration files are system specific but if you need to
simulate a single arm (kinematic simulation only so far), you can use
one of the `console-XXX_KIN_SIMULATED.json` file.  This should likely
be used with the `dvrk-ros`/`dvrk_robot` ROS package to start RViz
with the proper CAD models.  All shared console configuration files
are in `console`.

## ROS IO

These files can be used to configure the `dvrk_robot`/`dvrk_console_json` ROS application to publish the IO data of one or more arms.  See the `dvrk-ros`/`dvrk_robot` package.

## sawSocketStreamer and sawOpenIGTLink

Some examples of configuration files and instructions for *sawSocketStreamer* (streaming out dVRK data over UDP in JSON format) and *sawOpenIGTLink* (*cisst/SAW* bridge for OpenIGTLink/igtl) can be find in the directory `socket-streamer` and `igtl`.  These can be used as middleware between the dVRK console and user applications.

## Obsolete files

* `dvmtm.rob` and `dvpsm.rob`.  Use the JSON files instead.  The `.rob` files can still be used with `robManipulator` in C++/Python if needed.
* `two-arms.json`.  These files were used with the old application teleop JSON.  This application has been replaced by the Qt console JSON and for ROS `dvrk_console_json`.  The "console" applications are far more convenient and configurable.  Search for existing `console-xyz.json` in this directory and subdirectory to find different examples of use.
