# coding: utf-8

import time
import os
import subprocess
import multiprocessing

import sys
sys.path.insert(0, "/home/dvrk/catkin_ws/src/dvrk-ros/dvrk_python/src")
import dvrk

###temporary!!!!!!!!!!!!!!!!!!!###
subject_num=1;
trial_num=2;
scale=0.4;
###############################################
## for trials 1:24
	## if trials 1
		#clean
	## if trials 2:21...##
	
		## if control:

		##if radial 1Hz:

		#if radial 2.5 Hz:

		#if mixture:
	#if trial 22
		#clean
	#if trial 23
		#radial
	#if trial 24
		#mixture

#################################################
## Writing parameters for curerent trial
paramfile=open('/home/dvrk/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/bgu-dvrk/params.txt','w')
paramfile.write('1.5\n2.5')
paramfile.close()

## LAUNCH CONSOLE ##

# setup the dvrk ros environment
ros_env = os.environ 
ros_env["CATKIN_SHELL"] = "bash"
ros_env["ROSLISP_PACKAGE_DIRECTORIES"] = "/home/dvrk/catkin_ws/devel_release/share/common-lisp:/home/dvrk/catkin_ws/devel/share/common-lisp"
ros_env["PYTHONPATH"] = "/home/dvrk/catkin_ws/devel_release/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages"
ros_env["PKG_CONFIG_PATH"] = "/home/dvrk/catkin_ws/devel_release/lib/pkgconfig:/opt/ros/kinetic/lib/pkgconfig"
ros_env["CMAKE_PREFIX_PATH"] = "/home/dvrk/catkin_ws/devel_release:/home/dvrk/catkin_ws/devel:/opt/ros/kinetic"

os.chdir("/home/dvrk/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/bgu-dvrk/")

# launch the dvrk console via rosrun (via external process)
# subprocess.Popen("rosrun dvrk_robot dvrk_console_json -j console-MTMR-PSM1-MTML-PSM2-Teleop-with-cam-yarden.json", shell=True, env=ros_env, stdout=subprocess.PIPE).communicate()

devnull = open(os.devnull, "w")
#process_console = subprocess.Popen("rosrun dvrk_robot dvrk_console_json -j console-MTMR-PSM1-MTML-PSM2-Teleop-with-cam-yarden.json", shell=True, env=ros_env, stdout=devnull, stderr=devnull) # , stdout=subprocess.PIPE)

#print('dvrk console started')
console = dvrk.console()
# do velocity checks before!!
print('homing')
console.home()

print('setting scale to'+str(scale))
console.teleop_set_scale(scale)

# setup pre-trial (location of PSMs, etc)###################################



# wait for user to start experiment
raw_input("Press any key to start teleoperation and recording...")
print('starting teleoperation and recording')
console.teleop_start()
# start recording rosbag
#process_rosbag = subprocess.Popen("rosbag record -O /home/dvrk/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/bgu-dvrk/bag_files/subject"+str(subject_num)+"_trial"+ str(trial_num)+".bag -a", shell=True, env=ros_env, stdout=devnull, stderr=devnull)
process_rosbag = subprocess.Popen("rosbag record -a", shell=True, env=ros_env, stdout=devnull, stderr=devnull)
#wait for user input -- finish trial
raw_input("Press any key to stop teleoperation and recording...")##???
print('stop teleoperation')
console.teleop_stop()

# close rosbag
print('stop recording')
pid_string_of_rosbag_record=subprocess.Popen('''ps -ax | grep -F "/opt/ros/kinetic/lib/rosbag/record" | grep -v 'grep' | awk '{print $1}' ''', shell=True, stdout=subprocess.PIPE).communicate()[0].rstrip()
subprocess.Popen("kill -2 " + str(pid_string_of_rosbag_record), shell=True)
#terminate_ros_node("/record")

raw_input("Press any key to power off and close console")##???
print('powering off')
console.power_off()

## close the experiment
#print('close the experiment console')
#process_console.terminate()
#process_console.wait()
# also close the dvrk_console_json process 
#pid_string_of_console_gui=subprocess.Popen('''ps -ax | grep -F "/home/dvrk/catkin_ws/devel_release/lib/dvrk_robot/dvrk_console_json" | grep -v 'grep' | awk '{print $1}' ''', shell=True, stdout=subprocess.PIPE).communicate()[0].rstrip()
#subprocess.Popen("kill -9 " + str(pid_string_of_console_gui), shell=True)




print('Done')

# wrap up (stop recording, close terminal, etc)
devnull.close()


##32016 pts/19   S+     0:00 /bin/sh -c rosrun dvrk_robot dvrk_console_json -j console-MTMR-PSM1-MTML-PSM2-Teleop-with-cam-yarden.json
##32017 pts/19   Sl+    0:47 /home/dvrk/catkin_ws/devel_release/lib/dvrk_robot/dvrk_console_json -j console-MTMR-PSM1-MTML-PSM2-Teleop-with-cam-yarden.json



