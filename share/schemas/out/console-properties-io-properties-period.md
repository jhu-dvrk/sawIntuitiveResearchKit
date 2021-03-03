# Untitled number in dVRK-console Schema

```txt
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/period
```

Periodicity for the IO and PID loop.  Defined in seconds, i.e. interval between two iterations.  The default is defined in `mtsIntuitiveResearchKit.h` and is set so the loop frequency is 1500Hz, \~0.66ms so 0.00066s.  This is an advanced setting and most users should avoid defining it.

| Abstract            | Extensible | Status         | Identifiable            | Custom Properties | Additional Properties | Access Restrictions | Defined In                                                         |
| :------------------ | :--------- | :------------- | :---------------------- | :---------------- | :-------------------- | :------------------ | :----------------------------------------------------------------- |
| Can be instantiated | No         | Unknown status | Unknown identifiability | Forbidden         | Allowed               | none                | [console.schema.json*](console.schema.json "open original schema") |

## period Type

`number`

## period Constraints

**minimum (exclusive)**: the value of this number must be greater than: `0`
