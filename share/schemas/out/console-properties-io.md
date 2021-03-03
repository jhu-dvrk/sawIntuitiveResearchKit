# Untitled object in dVRK-console Schema

```txt
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io
```



| Abstract            | Extensible | Status         | Identifiable | Custom Properties | Additional Properties | Access Restrictions | Defined In                                                         |
| :------------------ | :--------- | :------------- | :----------- | :---------------- | :-------------------- | :------------------ | :----------------------------------------------------------------- |
| Can be instantiated | No         | Unknown status | No           | Forbidden         | Forbidden             | none                | [console.schema.json*](console.schema.json "open original schema") |

## io Type

`object` ([Details](console-properties-io.md))

# io Properties

| Property                                    | Type     | Required | Nullable       | Defined by                                                                                                                                                                                              |
| :------------------------------------------ | :------- | :------- | :------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [port](#port)                               | `string` | Optional | cannot be null | [dVRK-console](console-properties-io-properties-port.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/port")                               |
| [firewire-protocol](#firewire-protocol)     | `string` | Optional | cannot be null | [dVRK-console](console-properties-io-properties-firewire-protocol.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/firewire-protocol")     |
| [footpedals](#footpedals)                   | `string` | Optional | cannot be null | [dVRK-console](console-properties-io-properties-footpedals.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/footpedals")                   |
| [period](#period)                           | `number` | Optional | cannot be null | [dVRK-console](console-properties-io-properties-period.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/period")                           |
| [watchdog-timeout](#watchdog-timeout)       | `double` | Optional | cannot be null | [dVRK-console](console-properties-io-properties-watchdog-timeout.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/watchdog-timeout")       |
| [configuration-files](#configuration-files) | `array`  | Optional | cannot be null | [dVRK-console](console-properties-io-properties-configuration-files.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/configuration-files") |

## port

Port used communicate between the computer and the dVRK controllers.  Accepted values are "fw", "fw:X" (X is FireWire port), "udp" or "udp:xx.xx.xx.xx" (xx.xx.xx.xx is the IP address of the dVRK controller.  Default controller address is 169.254.0.100.  Values are parsed by `BasePort::ParseOptions` in library Amp1394.

`port`

*   is optional

*   Type: `string`

*   cannot be null

*   defined in: [dVRK-console](console-properties-io-properties-port.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/port")

### port Type

`string`

### port Constraints

**pattern**: the string must match the following regular expression: 

```regexp
^fw$|^fw:[0-9]$|^udp$|^udp:[0-9]{2,3}.[0-9]{2,3}.[0-9]{2,3}.[0-9]{2,3}
```

[try pattern](https://regexr.com/?expression=%5Efw%24%7C%5Efw%3A%5B0-9%5D%24%7C%5Eudp%24%7C%5Eudp%3A%5B0-9%5D%7B2%2C3%7D.%5B0-9%5D%7B2%2C3%7D.%5B0-9%5D%7B2%2C3%7D.%5B0-9%5D%7B2%2C3%7D "try regular expression with regexr.com")

## firewire-protocol

Protocol used for all FireWire communications.  This applies wether the controllers are connected to the computer using FireWired or Ethernet (UDP with Link-Local).  This is an advanced setting and most users should avoid defining it.

`firewire-protocol`

*   is optional

*   Type: `string`

*   cannot be null

*   defined in: [dVRK-console](console-properties-io-properties-firewire-protocol.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/firewire-protocol")

### firewire-protocol Type

`string`

### firewire-protocol Constraints

**enum**: the value of this property must be equal to one of the following values:

| Value                               | Explanation |
| :---------------------------------- | :---------- |
| `"sequential-read-write"`           |             |
| `"sequential-read-broadcast-write"` |             |
| `"broadcast-read-write"`            |             |

### firewire-protocol Default Value

The default value is:

```json
"sequential-read-broadcast-write"
```

## footpedals

IO configuration file used to define inputs from the foot pedals.  See files `sawRobotIO1394-*-foot-pedals.xml` in directory `share/io`.

`footpedals`

*   is optional

*   Type: `string`

*   cannot be null

*   defined in: [dVRK-console](console-properties-io-properties-footpedals.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/footpedals")

### footpedals Type

`string`

## period

Periodicity for the IO and PID loop.  Defined in seconds, i.e. interval between two iterations.  The default is defined in `mtsIntuitiveResearchKit.h` and is set so the loop frequency is 1500Hz, \~0.66ms so 0.00066s.  This is an advanced setting and most users should avoid defining it.

`period`

*   is optional

*   Type: `number`

*   cannot be null

*   defined in: [dVRK-console](console-properties-io-properties-period.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/period")

### period Type

`number`

### period Constraints

**minimum (exclusive)**: the value of this number must be greater than: `0`

## watchdog-timeout

Maximum laps of time allowed between to read/write access to the controller.  It is used to detect abnormally slow IOs or disconnected cables.  The value is sent to the controllers and the controllers will turn off power if the communication loop time exceeds the watchdog timeout.  Defined in seconds.  The default is defined in `mtsIntuitiveResearchKit.h` and is set to 30ms.  This is an advanced setting and most users should avoid defining it.  Setting it to zero disables the watchdog and should only be used by experts.  The maximum value is 300ms, i.e. 0.3s

`watchdog-timeout`

*   is optional

*   Type: `double`

*   cannot be null

*   defined in: [dVRK-console](console-properties-io-properties-watchdog-timeout.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/watchdog-timeout")

### watchdog-timeout Type

`double`

### watchdog-timeout Constraints

**maximum**: the value of this number must smaller than or equal to: `0.3`

**minimum**: the value of this number must greater than or equal to: `0`

## configuration-files



`configuration-files`

*   is optional

*   Type: `string[]`

*   cannot be null

*   defined in: [dVRK-console](console-properties-io-properties-configuration-files.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/configuration-files")

### configuration-files Type

`string[]`
