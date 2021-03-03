# Untitled string in dVRK-console Schema

```txt
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/port
```

Port used communicate between the computer and the dVRK controllers.  Accepted values are "fw", "fw:X" (X is FireWire port), "udp" or "udp:xx.xx.xx.xx" (xx.xx.xx.xx is the IP address of the dVRK controller.  Default controller address is 169.254.0.100.  Values are parsed by `BasePort::ParseOptions` in library Amp1394.

| Abstract            | Extensible | Status         | Identifiable            | Custom Properties | Additional Properties | Access Restrictions | Defined In                                                         |
| :------------------ | :--------- | :------------- | :---------------------- | :---------------- | :-------------------- | :------------------ | :----------------------------------------------------------------- |
| Can be instantiated | No         | Unknown status | Unknown identifiability | Forbidden         | Allowed               | none                | [console.schema.json*](console.schema.json "open original schema") |

## port Type

`string`

## port Constraints

**pattern**: the string must match the following regular expression: 

```regexp
^fw$|^fw:[0-9]$|^udp$|^udp:[0-9]{2,3}.[0-9]{2,3}.[0-9]{2,3}.[0-9]{2,3}
```

[try pattern](https://regexr.com/?expression=%5Efw%24%7C%5Efw%3A%5B0-9%5D%24%7C%5Eudp%24%7C%5Eudp%3A%5B0-9%5D%7B2%2C3%7D.%5B0-9%5D%7B2%2C3%7D.%5B0-9%5D%7B2%2C3%7D.%5B0-9%5D%7B2%2C3%7D "try regular expression with regexr.com")
