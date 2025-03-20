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

LCM.CreateAllAndWait(2.0)
LCM.StartAllAndWait(2.0)

# Now, look for arms
components = LCM.GetNamesOfComponents()
for comp in components:
  if comp.startswith('MTM') or comp.startswith('PSM') or comp.startswith('ECM'):
     print('Found ' + comp)
     obj = LCM.GetComponent(comp)
     comp_no_dash = comp.replace('-', '_')
     exec(f"{comp_no_dash} = obj")

print('System ready. Type dir(console) to see available commands.')
