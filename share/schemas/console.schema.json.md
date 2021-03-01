# dVRK-console

## Properties

- **`chatty`** *(boolean)*: Make the console say something stupid when it starts.  It's mostly a way to test the text-to-speech feature. Default: `False`.
- **`io`** *(object)*: Cannot contain additional properties.
  - **`firewire-protocol`** *(string)*: Protocol used for all FireWire communications.  This applies wether the controllers are connected to the computer using FireWired or Ethernet (UDP with Link-Local). Must be one of: `['sequential-read-write', 'sequential-read-broadcast-write', 'broadcast-read-write']`. Default: `sequential-read-broadcast-write`.
  - **`port`** *(string)*: Port used communicate between the computer and the dVRK controllers.  Accepted values are "fw", "fw:X" (X is FireWire port), "udp" or "udp:xx.xx.xx.xx" (xx.xx.xx.xx is the IP address of the dVRK controller.  Default controller address is 169.254.0.100.  Values are parsed by `BasePort::ParseOptions` in library Amp1394.
  - **`footpedals`** *(string)*: IO configuration file used to define inputs from the foot pedals.  See files `sawRobotIO1394-*-foot-pedals.xml` in directory `share/io`.
