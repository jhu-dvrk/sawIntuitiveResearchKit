# Introduction

The goal of this plugin is to create a cisst/SAW component that has a "standard" cisstMultiTask arm interface (for the dVRK C++ stack) relying on ROS topics to communicate with the actual arm.  This can be used to replace a dVRK arm by any other robotic arm as long as the replacement arm has a ROS interface similar to the dVRK one.  This can also be used for applications that can't use a single process such as remote teleoperation over ethernet.  In this case, the remote arm can actually be a dVRK PSM.

# Usage

To run this, you need 2 PCs, one controlling your MTM(s) and one controlling your PSM(s).   The two PCs have to be configured to share a single rosmaster/roscore (google can help with that).

Then on the PSM PC, start `dvrk_robot dvrk_system -j system-PSMx.json`.  Nothing new on this side.  If you just want to test with a simulated PSM under RViz, you can use the existing launch file:
```sh
roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1
```

On the MTM+Teleop PC, start `dvrk_robot dvrk_system -j` with a system configuration file that looks like: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/devel/share/jhu-dVRK/system-MTML-PSM1_ROS-Teleop.json

At that point, you should be able to home both arms using the dVRK system for the MTM and PSM from ROS.  Then start teleoperation and the simulated PSM should be controlled by your MTM.
