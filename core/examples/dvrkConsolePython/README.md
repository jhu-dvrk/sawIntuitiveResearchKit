# dVRK Console Python example

This example can be used either with a standalone Python interpreter or with an embedded Python
interpreter (e.g., embedded in a C++ application). Python 2.7 and Python 3 are both supported.

## Standalone Python interpreter

Start your favorite Python shell (e.g., `python` or `ipython`).
From the Python prompt, type: `from dvrk_console import *`

## Embedded Python interpreter

You can use the embedded Python interpreter provided by `ipython` or `wxPython`.
In the latter case, wxPython 4+ must be installed in your Python distribution (for Python 2.7,
use wxPython 4.1.0).

In either case, the embedded Python interpreter can be dynamically loaded by the component
manager in the `sawIntuitiveResearchKitSystem` executable. The recommended approach
is to create a separate JSON file with the following contents (for `ipython`):

```
{
    "components":
    [
        {
            "shared-library": "cisstInteractive",
            "class-name": "ireTask",
            "constructor-arg": {
                "Name": "IRE",
                "Shell": "IRE_IPYTHON",
                "Startup": "from dvrk_console import *"
            }
        }
    ]
}
```

To use `wxPython`, change `Shell` to `IRE_WXPYTHON`. Assuming the above file is called
`ire-ipython.json`, it can be invoked using the `-m` flag:

```
sawIntuitiveResearchKitSystem  -j  <your_config>.json  -m  ire-ipython.json
```

where `<your_config>.json` is the JSON file that specifies your system configuration (e.g.,
which MTMs, PSMs, ECM are used).

## Troubleshooting

  - If Python fails to find the cisst Python libraries, make sure the `PYTHONPATH` environment
variable is set up correctly, e.g., using `cisstvars.sh` (Linux) or `cisstvars.bat` (Windows).

  - If `dvrk_console.py` is not found, either specify the full path when importing the file,
or copy it to one of the directories in `PYTHONPATH`.

  - Currently, `dvrk_console.py` determines that it running in an embedded Python interpreter
if it finds a component named `IRE`. Make sure to use that name in your JSON file, as in the example above.

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
