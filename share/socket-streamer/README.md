# sawIntuitiveResearchKit with sawSocketStreamer

These configuration files show how to use sawSocketStreamer with the dVRK.  This code
doesn't require ROS (use `sawIntuitiveResearchKitQtConsoleJSON`
instead of the ROS node `dvrk_robot dvrk_console_json`) so it should
run fine on all OSs supported by both the dVRK and *cisst* `osaSocket`.

# Links
  * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
  * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
  * sawIntuitiveResearchKit: https://github.com/jhu-dvrk/sawIntuitiveResearchKit
  * sawSocketStreamer: https://github.com/jhu-saw/sawSocketStreamer


# Running the examples

## Dynamic loading

You will need two extra configuration files (examples can be found in the `share/socket-streamer` directory):
* Component manager configuration file to load the dynamic library `sawSocketStreamer` and create/configure the socket streamer bridge.  The cisst component manager creates an instance of the class `mtsSocketStreamer` and connects the streamer to an existing component/interface (e.g., `PSM1/Arm`).  The component manager configuration file is passed to the main program using the option `-m`.
* Socket streamer configuration file.  This indicates which commands and types to stream out (e.g. `measured_js/prmStateJoint`).

## Running the code and testing

Go in the directory with the socket streamer configuration files (i.e., the directory containing this README).  Then launch a dVRK console with a simulated PSM:
```sh
rosrun dvrk_robot dvrk_console_json -j  ../console/console-PSM1_KIN_SIMULATED.json -m manager-socket-streamer-PSM1.json
```

At that point the dVRK can send messages over UDP in JSON format.  To test this on Linux, home the simulated PSM using the dVRK GUI and use the `nc` command line tool:
```sh
nc -lu 127.0.0.1 48051
```

## Output

The data is sent in JSON format.  For example:
```json
{
  "gripper/measured_js": {
    "AutomaticTimestamp":true,
    "Effort":null,
    "Name":["gripper"],
    "Position":[4.6701996844274162e-310],
    "Timestamp":19.699493094000001,
    "Valid":false,
    "Velocity":null
  },
  "measured_cp": {
    "AutomaticTimestamp":false,
    "MovingFrame":"MTML",
    "Position": {
      "Rotation": [[3.6732051033050439e-06,0.99999999996626898,7.3464102068321324e-06],
                   [-2.6984803280782899e-11,-7.3464102068321324e-06,0.99999999997301525],
                   [0.99999999999325384,-3.6732051035270885e-06,0]],
      "Translation": [-1.3388794904003909e-06,-0.36449897370403511,0.12879999999811512]
    },
    "ReferenceFrame": "MTML_base",
    "Timestamp":19.699094908999999,
    "Valid":true
  },
  "measured_js": {
    "AutomaticTimestamp":false,
    "Effort":[0,0,0,0,0,0,0],
    "Name":["outer_yaw","shoulder_pitch","elbow_pitch","wrist_platform","wrist_pitch","wrist_yaw","wrist_roll"],
    "Position":[0,0,0,0,0,0,0],
    "Timestamp":19.699094908999999,
    "Valid":true,
    "Velocity":[0,0,0,0,0,0,0]
  }
}
```

Output has been reformated for readability, actual output has no carriage returns or spaces.
