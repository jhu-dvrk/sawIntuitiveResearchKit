Arm configuration files with no hardware dependencies
=====================================================

This directory contains dVRK arm configuration files for arms without any hardware dependencies, i.e. simulated arms.
These are used by the console configuration files provided in the `share/console` directory but you can also include them in your own console configuration files (e.g. use a real MTM to tele-operate a simulated PSM).   The suffix `KIN_SIMULATED` indicates that we use a purely position based simulation, i.e. no dynamic simulation nor interactions with the rest of the world.

* `ECM_KIN_SIMULATED_STRAIGHT.json`: simulated ECM with a straight endoscope, you can change the endoscope type at runtime using ROS topics or the Qt GUI.
* `MTML_KIN_SIMULATED.json`: simulated MTML
* `MTMR_KIN_SIMULATED.json`: simulated MTMR
* `PSM_KIN_SIMULATED_MANUAL.json`: simulated PSM with `MANUAL` tool detection.  You will have to use the Qt GUI or ROS topics to set the tool type at runtime.  This allows to simulate tool swapping.
* `PSM_KIN_SIMULATED_LARGE_NEEDLE_DRIVER_400006.json`: simulated PSM with fixed tool detection (i.e. hard coded to `LARGE_NEEDLE_DRIVER_400006`,  the tool type can't be changed at runtime.
* `suj-simulated.json`: simulated SUJ, i.e. all 4 Setup Joints arms on the patient cart but without any active arm (PSMs and ECM).  To simulate the whole patient cart, see `share/console/console-full-cart-simulated`.

