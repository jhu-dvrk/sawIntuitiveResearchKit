# Create and run the sawIntuitiveResearchKitQtPID application using Python.
# From command line:  python -i sawIntuitiveResearchKitQtPID.py
# From Python:  from sawIntuitiveResearchKitQtPID import *

# Following is needed for Linux
import sys, dl
sys.setdlopenflags(dl.RTLD_GLOBAL | dl.RTLD_LAZY) 

from cisstMultiTaskPython import *
LCM = mtsManagerLocal_GetInstance()

# Create a dummy component to get access to manager component services
ManagerComponent = mtsComponentWithManagement('Manager')
LCM.AddComponent(ManagerComponent)

LCM.CreateAll()
LCM.StartAll()

# Get an interface to the manager component services
Manager = ManagerComponent.GetManagerComponentServices()

print 'Loading shared libraries'
Manager.Load('cisstMultiTaskQt')
Manager.Load('sawRobotIO1394')
Manager.Load('sawRobotIO1394Qt')
Manager.Load('sawControllers')
Manager.Load('sawControllersQt')

print 'Creating and starting component viewer'
Manager.ComponentCreate('mtsComponentViewer', 'Viewer')
Manager.ComponentStart('Viewer')

print 'Creating Qt application'
Manager.ComponentCreate('mtsQtApplication', 'QtApp QtApp')

#Create Qt widget (hard-coded to 8 actuators)
print 'Creating Robot I/O Qt widget'
Manager.ComponentCreate('mtsRobotIO1394QtWidget', 'ioGUI')
Manager.ComponentConfigure('ioGUI', ' ')

# Create task with 0.001 sec (1 ms) period, using firewire port 0 (hard-coded)
print 'Creating Robot I/O periodic task'
Manager.ComponentCreate('mtsRobotIO1394', mtsTaskPeriodicConstructorArg('io', 0.001))
Manager.ComponentConfigure('io', 'sawRobotIO1394-MTML.xml')

print 'Creating PID Qt widget'
Manager.ComponentCreate('mtsPIDQtWidget', 'pidGUI')
Manager.ComponentConfigure('pidGUI', ' ')

print 'Creating PID task'
Manager.ComponentCreate('mtsPID', mtsTaskPeriodicConstructorArg('pid', 0.001))
Manager.ComponentConfigure('pid', 'sawControllersPID-MTML.xml')

print 'Connecting components'
Manager.Connect('ioGUI', 'Robot', 'io', 'MTML')
Manager.Connect('ioGUI', 'RobotActuators',  'io', 'MTMLActuators')
Manager.Connect('pidGUI', 'Controller', 'pid', 'Controller')
Manager.Connect('pid', 'ExecIn', 'io', 'ExecOut')
Manager.Connect('pid', 'RobotJointTorqueInterface', 'io', 'MTML')


print 'Starting components'
Manager.ComponentStart('io')
Manager.ComponentStart('ioGUI')
Manager.ComponentStart('pid')
Manager.ComponentStart('pidGUI')
Manager.ComponentStart('QtApp')
