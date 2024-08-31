# Description

This is a simple Qt based application to provide a couple of widgets to be displayed in the master console.  The two widgets (one for each eye/monitor, left and right) display the current state of the dVRK, including:
* Operator present
* Clutch pressed
* Is the teleoperation in camera mode
* With MTM/PSM pair is currently active (useful for users with multiple PSMs associated to a single MTM)
* State of the active MTM/PSM teleoperation(s)

# Usage

This application assumes that you're using your PC to send video signals to both monitors on the console stereo display (aka HRSV, high resolution stereo video) and you can add the widgets on top of the video you're rendering on the stereo console.

To start the application, use a launch file or `rosrun`:
```sh
rosrun dvrk_hrsv_widget hrsv_widget
```

Once the two widgets are opened, drag each of them manually towards the top (or bottom) center of each console monitor (left and right).  You might have to tweak their positions so they look fine in the console itself (same hight, proper disparity).
