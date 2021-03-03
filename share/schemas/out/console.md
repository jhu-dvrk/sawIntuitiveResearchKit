# dVRK-console Schema

```txt
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json
```



| Abstract            | Extensible | Status         | Identifiable | Custom Properties | Additional Properties | Access Restrictions | Defined In                                                        |
| :------------------ | :--------- | :------------- | :----------- | :---------------- | :-------------------- | :------------------ | :---------------------------------------------------------------- |
| Can be instantiated | No         | Unknown status | No           | Forbidden         | Allowed               | none                | [console.schema.json](console.schema.json "open original schema") |

## dVRK-console Type

`object` ([dVRK-console](console.md))

# dVRK-console Properties

| Property          | Type      | Required | Nullable       | Defined by                                                                                                                                        |
| :---------------- | :-------- | :------- | :------------- | :------------------------------------------------------------------------------------------------------------------------------------------------ |
| [io](#io)         | `object`  | Optional | cannot be null | [dVRK-console](console-properties-io.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io")         |
| [chatty](#chatty) | `boolean` | Optional | cannot be null | [dVRK-console](console-properties-chatty.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/chatty") |

## io



`io`

*   is optional

*   Type: `object` ([Details](console-properties-io.md))

*   cannot be null

*   defined in: [dVRK-console](console-properties-io.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/io")

### io Type

`object` ([Details](console-properties-io.md))

## chatty

Make the console say something useless when it starts.  It's mostly a way to test the text-to-speech feature.

`chatty`

*   is optional

*   Type: `boolean`

*   cannot be null

*   defined in: [dVRK-console](console-properties-chatty.md "https://github.com/jhu-dvrk/sawIntuitiveResearchKit/schemas/console.schema.json#/properties/chatty")

### chatty Type

`boolean`
