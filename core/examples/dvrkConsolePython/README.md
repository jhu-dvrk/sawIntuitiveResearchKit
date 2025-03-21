# dVRK Console Python example

## Usage

From Python prompt, type: `from dvrk_console import *`

## Finding Commands

All commands are available via (required) interfaces that are automatically created for
the console and any configured arms (MTM, PSM or ECM). These can be found by typing `dir()`.
For example, `dir(PSM1)` will list all available commands for the PSM1 interface.
Note that the system automatically replaces dashes ('-') with underscores ('_') in the interface name.

Some cisst components use namespaces to organize the commands, using the forward slash ('/')
as a separator. For example, `jaw/measured_js` corresponds to the `measured_js` command in
the `jaw` namespace. Since these would not be valid names in Python, the system converts them to
nested classes. Thus, if `jaw/measured_js` is in the `PSM1` interface, it can be invoked in
Python by typing `PSM1.jaw.measured_js()`.

The script also creates a `dvrk` Python dictionary (`dict` type), which is a collection of
dictionaries for each interface. The contents of this dictionary can easily be displayed using the
`dvrk_list` function:

```
dvrk_list()            # List all commands in dvrk dictionary, organized by interface
dvrk_list('PSM1')      # List all keys for the PSM1 interface
dvrk_list('PSM1-PID')  # List all keys for the PSM1-PID interface
```

Unlike the interface variable name (e.g., `PSM1_PID`), the keys in the dvrk dictionary
do not have any character replacements (e.g., it will be `PSM1-PID`).

## Determine Parameter Types

One method for determining the data type required by a command is to invoke the `GetArgumentPrototype`
method on the command.  For example,

```
arg = PSM1.servo_jp.GetArgumentPrototype()
```

In the above example, `arg` will be of type `prmPositionJointSet` and can be updated and then
passed as a parameter to `servo_jp`. For example, the following will move all six PSM1 joints to 0:

```
goal = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
arg.SetGoal(goal)
PSM1.servo_jp(arg)
```
