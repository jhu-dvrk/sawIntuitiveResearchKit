# CRTK Port

For users porting their code from dVRK 1.7, we provide a simple script to query/replace symbols as well as files with old names and new names.  The script will not be able fully port your code but will likely help.
See https://github.com/jhu-cisst/cisst/tree/devel/utils/crtk-port

* For C++ code using *cisstMultiTask* commands and events, use the dictionary files:
  * `~/catkin_ws/src/cisst-saw/cisst/utils/crtk-port/crtk-commands.dict`
  * `~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/crtk-port/members-saw-intuitive-research-kit.dict`
 
* For ROS clients, you can rename most of the topics used with the file: `~/catkin_ws/src/cisst-saw/cisst/utils/crtk-port/crtk-ros-commands.dict`
