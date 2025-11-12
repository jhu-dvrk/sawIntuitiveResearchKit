# Introduction

This directory is an example of extending the default teleoperation component to support four-channel bilateral teleoperation between a PSM and a MTM. It can either be used as-is for teleoperation, or used as a starting point for customizing the teleoperation in some way.

If you want to use this as-is, skip to Usage. Otherwise, it is recommended to copy/paste the content of this directory in a different location in your ROS workspace (under `src`), and rename the components and library it creates to prevent conflicts with this example.

# Using as a template

In order to use this example as a template for your own custom teleoperation behavior, first create of copy of this example directory somewhere else in your ROS workspace (inside the `src/` folder). After creating a copy as a starting point, you will need to rename the package and library it creates, or else it will conflict with this example. You need to change the package `name` inside both `package.xml` and `colcon.pkg`. Next, in `CMakeLists.txt` change the project name (everywhere that `saw_intuitive_research_kit_example_bilateral_teleop` appears), as well as the name of the library produced (everywhere that `sawIntuitiveResearchKitBilateralTeleop` appears).

Once you've created a copy with a new package/library name, you are free to begin modifying the code in the copy as you wish. Note: when starting a console as described in Usage below, make sure you use the name of your re-named library, not the name of this example.

# Usage

To use the teleoperation component, we simply need need to create a dVRK console configuration file for it. See `share/system-MTML-PSM2-bilateral-teleop.json` for an example of how to do this (make sure you put the console config file with your usual console config files or the dVRK won't be able to find your arm config files). The `classname` and `shared-library` fields tell the dVRK where to find the new teleop component, so if you are modifying this example also make sure they match your package.

It is recommend you tune the PID disturbance observers for the PSM and MTM used in bilateral teleoperation (or simply disable them). This can be done by editing `sawIntuitiveResearchKit/share/pid/sawControllersPID-<arm type>.json`, and replace `use_disturbance_observer: true` with `use_disturbance_observer: false` for each actuator (or tune the disturbance observer).

The example component can be switched on the fly between bilateral and unilateral mode via the cisstMultitask command `set_bilateral_enabled`. In order to change modes via ROS, we need to load a ROS bridge by adding `-m <path to file>/<ROS bridge manager config>` to the end of the command when you start the dVRK console. An example is given in `share/manager-MTML-PSM2-bilateral-ros.json`, make sure the `configure-parameter` matches the teleop component you want to control via ROS. If you want add ROS bridges for multiple bilateral teleop components, simply add another bridge component to the manager config file.

# Config options

The bilateral teleop component supports all the same configuration options are the default one, as well as a few more described below. Note that since the component is created by the component manager, the teleop config should go into a separate file as shown in the example, and the component's `configure-parameter` should be set to the name of that config file.

`mtm_torque_gain` (floating point, default 0.2): scale down torque forces applied to MTM, used to reduce some oscillation/instability that is frequently present

`psm_force_source`: format `{ "component": <component name>, "interface": <name>, "function": <name> }`, used to specify a cisst component to get force estimation from for the PSM (by default, `measured_cf` from motor torques will be used). For example, a force sensor or a custom component running a dynamics model. The read command function should return type `prmForceCartesianGet`. In addition, you will need to add an explicit connection to the component manager file (e.g. `manager-MTML-PSM2-bilteral-ros.json` in the example) to connect the force source interface (e.g. `ATI_gamma:force_sensor`) to the teleop component's corresponding interface (e.g. `MTML_PSM2:ATI_gamma_force_sensor_force_source`). The teleop's corresponding interface will be named `<source component name>_<source interface_name>_force_source`. The required connection is illustrated in the example configuration.

`mtm_force_source`: same as `psm_force_source`, but provides estimation of forces on the MTM.
