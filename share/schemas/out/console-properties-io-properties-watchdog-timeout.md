# Untitled double in dVRK-console Schema

```txt
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/watchdog-timeout
```

Maximum laps of time allowed between to read/write access to the controller.  It is used to detect abnormally slow IOs or disconnected cables.  The value is sent to the controllers and the controllers will turn off power if the communication loop time exceeds the watchdog timeout.  Defined in seconds.  The default is defined in `mtsIntuitiveResearchKit.h` and is set to 30ms.  This is an advanced setting and most users should avoid defining it.  Setting it to zero disables the watchdog and should only be used by experts.  The maximum value is 300ms, i.e. 0.3s

| Abstract            | Extensible | Status         | Identifiable            | Custom Properties | Additional Properties | Access Restrictions | Defined In                                                         |
| :------------------ | :--------- | :------------- | :---------------------- | :---------------- | :-------------------- | :------------------ | :----------------------------------------------------------------- |
| Can be instantiated | No         | Unknown status | Unknown identifiability | Forbidden         | Allowed               | none                | [console.schema.json*](console.schema.json "open original schema") |

## watchdog-timeout Type

`double`

## watchdog-timeout Constraints

**maximum**: the value of this number must smaller than or equal to: `0.3`

**minimum**: the value of this number must greater than or equal to: `0`
