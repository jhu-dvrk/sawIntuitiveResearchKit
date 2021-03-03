# Untitled string in dVRK-console Schema

```txt
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io/properties/firewire-protocol
```

Protocol used for all FireWire communications.  This applies wether the controllers are connected to the computer using FireWired or Ethernet (UDP with Link-Local).  This is an advanced setting and most users should avoid defining it.

| Abstract            | Extensible | Status         | Identifiable            | Custom Properties | Additional Properties | Access Restrictions | Defined In                                                         |
| :------------------ | :--------- | :------------- | :---------------------- | :---------------- | :-------------------- | :------------------ | :----------------------------------------------------------------- |
| Can be instantiated | No         | Unknown status | Unknown identifiability | Forbidden         | Allowed               | none                | [console.schema.json*](console.schema.json "open original schema") |

## firewire-protocol Type

`string`

## firewire-protocol Constraints

**enum**: the value of this property must be equal to one of the following values:

| Value                               | Explanation |
| :---------------------------------- | :---------- |
| `"sequential-read-write"`           |             |
| `"sequential-read-broadcast-write"` |             |
| `"broadcast-read-write"`            |             |

## firewire-protocol Default Value

The default value is:

```json
"sequential-read-broadcast-write"
```
