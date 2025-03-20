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

# Create a dictionary for useful dVRK commands.
# It takes advantage of that fact that currently, most or all useful commands
# are lower-case.
dvrk = dict()

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
    console = cisstMultiTask.mtsCreateClientInterface('dvrkClient', 'dvrkServer', 'Main')
    print('Connecting internal required interfaces')
    console.connect()
    for command in dir(console):
       if command.islower() and not command.startswith('_') and not command.startswith('this'):
          dvrk['console/'+command] = getattr(console, command)

print('Console ready. Type dir(console) to see available commands.')

# Now, look for arms
dvrkClient = LCM.GetComponent('dvrkClient')
components = LCM.GetNamesOfComponents()
for comp in components:
  if comp.startswith('MTM') or comp.startswith('PSM') or comp.startswith('ECM'):
     print('Found ' + comp)
     obj = LCM.GetComponent(comp)
     provInterfaces = obj.GetNamesOfInterfacesProvided()
     for prov in provInterfaces:
         if (prov == 'Controller') or (prov == 'Arm'):
             comp_no_dash = comp.replace('-', '_')
             interface = cisstMultiTask.mtsCreateClientInterface(comp_no_dash+'Client', comp, prov)
             exec(f"{comp_no_dash} = interface")
             print('Type dir(' + comp_no_dash + ') to see available commands.')
             for command in dir(interface):
                 if command.islower() and not command.startswith('_') and not command.startswith('this'):
                     dvrk[comp+'/'+command] = getattr(interface, command)
             break

LCM.CreateAllAndWait(2.0)
LCM.StartAllAndWait(2.0)

print('System ready. See dvrk dict.')
