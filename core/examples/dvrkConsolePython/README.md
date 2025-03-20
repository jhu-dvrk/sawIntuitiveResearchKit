# dVRK Console Python example

## Usage

From Python prompt, type: `from dvrk_console import *`

## Finding Commands

All commands are available via (required) interfaces that are automatically created for
the console and any configured arms (MTM, PSM or ECM). These can be found by typing `dir()`.
For example, `dir(PSM1)` will list all available commands for the PSM1 interface.
Note that the system automatically replaces dashes ('-') with underscores ('_') in the interface name.

The script also creates a `dvrk` Python dictionary (`dict` type). The contents of this dictionary
can easily be displayed using the `dvrk_list` function:

```
dvrk_list()          # List all keys in dvrk dictionary
dvrk_list('PSM1')    # List all keys starting with PSM1 (e.g., PSM1 and PSM1-PID)
dvrk_list('PSM1/')   # List all keys starting with PSM1/ (e.g., not PSM1-PID)
```

Unlike the interface variable name (e.g., PSM1_PID), the keys in the dvrk dictionary
do not have any character replacements (e.g., it will be PSM1-PID). Also, the dictionary only
contains commands with lower-case characters, which eliminates the internal commands.


