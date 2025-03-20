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
             print('Use getattr(' + comp_no_dash + ', \'one/two\')() to run command that has namespace (/).')
             break

LCM.CreateAllAndWait(2.0)
LCM.StartAllAndWait(2.0)

print('System ready.')
