# coding: utf-8
import subprocess, os

# setup the dvrk ros environment
ros_env = os.environ 
ros_env["CATKIN_SHELL"] = "bash"
ros_env["ROSLISP_PACKAGE_DIRECTORIES"] = "/home/dvrk/catkin_ws/devel_release/share/common-lisp:/home/dvrk/catkin_ws/devel/share/common-lisp"
ros_env["PYTHONPATH"] = "/home/dvrk/catkin_ws/devel_release/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages"
ros_env["PKG_CONFIG_PATH"] = "/home/dvrk/catkin_ws/devel_release/lib/pkgconfig:/opt/ros/kinetic/lib/pkgconfig"
ros_env["CMAKE_PREFIX_PATH"] = "/home/dvrk/catkin_ws/devel_release:/home/dvrk/catkin_ws/devel:/opt/ros/kinetic"

os.chdir("/home/dvrk/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/bgu-dvrk/")

# launch the dvrk console via rosrun (via external process)
subprocess.Popen("rosrun dvrk_robot dvrk_console_json -j console-MTMR-PSM1-MTML-PSM2-Teleop-with-cam-yarden.json", shell=True, env=ros_env, stdout=subprocess.PIPE).communicate()

