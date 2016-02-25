VRK configuration files
=======================

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
