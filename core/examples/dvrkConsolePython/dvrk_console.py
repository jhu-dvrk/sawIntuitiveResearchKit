# Start dVRK console

import os, sys, ctypes

# Set RTLD_GLOBAL flag for dynamic loading (on Linux)
try:
   flags = sys.getdlopenflags()
   sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)
except AttributeError as e:
    print('Skipping dlopen flags, ' + str(e))

import cisstCommonPython as cisstCommon

# Set up cisst logging system to print errors, warnings, and verbose (but not debug)
cisstCommon.cmnLogger.SetMask(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.SetMaskFunction(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.SetMaskDefaultLog(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.AddChannelToStdOut(cisstCommon.CMN_LOG_ALLOW_ERRORS_AND_WARNINGS)

def log():
   os.system('tail cisstLog.txt')

import cisstMultiTaskPython as cisstMultiTask
import cisstParameterTypesPython as cisstParameterTypes
import numpy

# Create a dictionary for dVRK commands, organized by interfaces
# (i.e., this is a dictionary of dictionaries)
dvrk = dict()

def dvrk_list(req_key = ''):
    def dvrk_list_inner(cur_dict, indent, req_key=''):
        for key, value in cur_dict.items():
            if not req_key or (key == req_key):
                if isinstance(value, dict):
                    print(indent+key+'/')
                    dvrk_list_inner(value, indent+'    ', '')
                elif isinstance(value, cisstMultiTask.mtsInterfaceRequiredPython):
                    print(indent+key+'/')
                    dvrk_list_inner(value.__dict__, indent+'    ', '')
                else:
                    print(indent+key)
    dvrk_list_inner(dvrk, '', req_key)

# Set up the console interface and add commands to the dvrk dictionary
def SetupConsole(serverName):
    console = cisstMultiTask.mtsCreateClientInterface('dvrkClient', serverName, 'Main')
    print('Connecting internal required interfaces')
    console.connect()
    console_dict = dict()
    for command in dir(console):
       # Ignore commands that start with '_' or 'this'
       if not command.startswith('_') and not command.startswith('this'):
          console_dict[command] = getattr(console, command)
    dvrk['console'] = console_dict
    return console

# Find arm components, add interfaces, and add commands to dvrk dictionary
def SetupArms(components):
    for comp in components:
        if comp.startswith('MTM') or comp.startswith('PSM') or comp.startswith('ECM'):
            print('Found ' + comp)
            obj = LCM.GetComponent(comp)
            provInterfaces = obj.GetNamesOfInterfacesProvided()
            for prov in provInterfaces:
                if (prov == 'Controller') or (prov == 'Arm'):
                    comp_no_dash = comp.replace('-', '_')
                    interface = cisstMultiTask.mtsCreateClientInterface(comp_no_dash+'Client', comp, prov)
                    setattr(sys.modules[__name__], comp_no_dash, interface)
                    print('Type dir(' + comp_no_dash + ') to see available commands.')
                    arm_dict = dict()
                    for command in dir(interface):
                        # Ignore commands that start with '_' or 'this'
                        if not command.startswith('_') and not command.startswith('this'):
                            arm_dict[command] = getattr(interface, command)
                    dvrk[comp] = arm_dict
                    break

LCM = cisstMultiTask.mtsManagerLocal.GetInstance()
LCM.CreateAll()
LCM.StartAll()

dvrkServer = cisstMultiTask.mtsLoadAndCreateServer('sawIntuitiveResearchKit',
                                                   'mtsIntuitiveResearchKitConsole',
                                                   'dvrkServer', '')
if dvrkServer:
    print('Configuring dVRK server.')
    # Python2 uses raw_input and Python3 uses input
    try:
        configFile = raw_input('Enter config filename (JSON): ')
    except NameError:
        configFile = input('Enter config filename (JSON): ')
    dvrkServer.Configure(configFile)
    # Setup up console component, which will create other components,
    # including the arms (MTM, PSM, ECM)
    console = SetupConsole('dvrkServer')
    # Now, look for the arm components (MTM, PSM, ECM)
    SetupArms(LCM.GetNamesOfComponents())

LCM.CreateAllAndWait(2.0)
LCM.StartAllAndWait(2.0)

print('System ready. See dvrk dict.')
