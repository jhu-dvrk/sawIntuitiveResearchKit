# sawIntuitiveResearchKit with sawOpenIGTLink

These configuration files show how to use sawOpenIGTLink with the dVRK.  This code
doesn't require ROS (use `sawIntuitiveResearchKitQtConsoleJSON`
instead of the ROS node `dvrk_robot dvrk_console_json`) so it should
run fine on all OSs supported by both the dVRK and IGTL.

# Links
  * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
  * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
  * sawIntuitiveResearchKit: https://github.com/jhu-dvrk/sawIntuitiveResearchKit
  * sawOpenIGTLink: https://github.com/jhu-saw/sawOpenIGTLink


# Running the examples

## Dynamic loading

you will need two extra configuration files (examples can be found in `share/igtl` directory):
* Component manager configuration file to load the dynamic library `sawOpenIGTLink` and create/configure the OpenIGTLink bridge.  The cisst component manager creates an instance of the class `mtsIGTLCRTKBridge`.  The component manager configuration file is passed to the main program using the option `-m`.
* IGTL bridge configuration file.  This indicates which component and interface to bridge to IGTL.  For the dVRK, the component name is the arm name (e.g. PSM1) and the interface name is always "Arm"

## Running the code and testing

Go in the directory with the igtl configuration files (i.e. the directory containing this README).  Then launch a dVRK console with a simulated PSM:
```sh
rosrun dvrk_robot dvrk_console_json -j  ../console/console-PSM1_KIN_SIMULATED.json -m manager-igtl-PSM1.json
```

At that point the dVRK can send/receive messages over IGTL.  To test this, home the arm (using the dVRK GUI) and use the utility provided along sawOpenIGTLink:
```sh
igtl_receive localhost 18944
```
