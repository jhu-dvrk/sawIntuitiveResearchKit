# dVRK-console

## Properties

- **`io`** *(object)*: Cannot contain additional properties.
  - **`port`** *(string)*: Port used communicate between the computer and the dVRK controllers.  Accepted values are "fw", "fw:X" (X is FireWire port), "udp" or "udp:xx.xx.xx.xx" (xx.xx.xx.xx is the IP address of the dVRK controller.  Default controller address is 169.254.0.100.  Values are parsed by `BasePort::ParseOptions` in library Amp1394.
  - **`firewire-protocol`** *(string)*: Protocol used for all FireWire communications.  This applies wether the controllers are connected to the computer using FireWired or Ethernet (UDP with Link-Local).  This is an advanced setting and most users should avoid defining it. Must be one of: `['sequential-read-write', 'sequential-read-broadcast-write', 'broadcast-read-write']`. Default: `sequential-read-broadcast-write`.
  - **`footpedals`** *(string)*: IO configuration file used to define inputs from the foot pedals.  See files `sawRobotIO1394-*-foot-pedals.xml` in directory `share/io`.
  - **`period`** *(number)*: Periodicity for the IO and PID loop.  Defined in seconds, i.e. interval between two iterations.  The default is defined in `mtsIntuitiveResearchKit.h` and is set so the loop frequency is 1500Hz, ~0.66ms so 0.00066s.  This is an advanced setting and most users should avoid defining it.
  - **`watchdog-timeout`** *(double)*: Maximum laps of time allowed between to read/write access to the controller.  It is used to detect abnormally slow IOs or disconnected cables.  The value is sent to the controllers and the controllers will turn off power if the communication loop time exceeds the watchdog timeout.  Defined in seconds.  The default is defined in `mtsIntuitiveResearchKit.h` and is set to 30ms.  This is an advanced setting and most users should avoid defining it.  Setting it to zero disables the watchdog and should only be used by experts.  The maximum value is 300ms, i.e. 0.3s. Minimum: `0.0`. Maximum: `0.3`.
  - **`configuration-files`** *(array)*
    - **Items** *(string)*
- **`chatty`** *(boolean)*: Make the console say something useless when it starts.  It's mostly a way to test the text-to-speech feature. Default: `False`.
