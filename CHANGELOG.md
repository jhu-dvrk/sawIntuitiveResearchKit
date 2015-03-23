Change log
==========

1.1.0 (2015-03-xx)
==================

* API changes:
  * SI units (meters) instead of millimeteres are now used for all translation joints (ECM and PSM joint 3).  Cartesian positions were always based on SI.  .rob files where already meter based.  Users want to pay attention to:
    * PID gains.  We provided new PID configurations files for all arms but if you have tuned gains for your system, you will need to increase them.
    * ROS joint topics.  If you used the ROS bridges, all joints were reported in millimeters.
  * A new base class has been introduced to share the code between all arms, now all arms are derived from `mtsIntuititiveResearchKitArm`.
* Deprecated features:
  * For all derived classes, the enum used to encode states are now shared across arm types, you will need to remove the ECM, MTM or PSM part (e.g. `DVRK_PSM_HOMING_POWERING` is now `DVRK_HOMING_POWERING`  
* New features:
  * All arms now feature four control modes:
    * Direct joint:  This mode checks current arm state to make sure it is powered.  Commands are sent to PID directly.
    * Direct cartesian: This mode checks current arm state to make sure is powered and tool (for ECM and PSM) is past cannula.  Inverse kinematics is computed and new joints goal sent to PID.
    * Joint goal: Same pre-condition as direct joit but the robLSPB trajectory generator is called to make sure the robot moves smoothly to goal.
    * Cartesian goal: Same pre-condition as direct cartesian but the inverse kinematic is calculated for current and goal positions then trajectory in sent as joint goal.   NOTE: interpolation is performed in joint space.
* Bug fixes:
  * Commands using the inverse kinematics now check the result of nmr

1.0.1 (2015-01-30)
==================

* Change log file created