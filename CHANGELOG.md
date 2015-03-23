Change log
==========

1.1.0 (2015-03-xx)
==================

* API changes:
  * SI units (meters) instead of millimeteres are now used for all translation joints (ECM and PSM joint 3).  Cartesian positions were always based on SI.  `.rob` files where already meter based.  Users want to pay attention to:
    * PID gains.  We provided new PID configurations files for all arms but if you have tuned gains for your system, you will need to increase them.
    * ROS joint topics.  If you used the ROS bridges, all joints were reported in millimeters.
  * Programmers using distances in the code should use the `cmn_mm` constant (e.g. `double tolerance = 1.0 * cmn_mm;`)
  * When building sawIntuitiveResearchKit, you must have cisst configured with `CISST_USE_SI_UNITS`.
  * Joint 7 on PSM is now referred as "jaw", e.g. `SetPositionJaw` instead of `angle` or `gripper`.  On MTM, the last joint is still called "gripper"
* Deprecated features:
  * For all derived classes, the enum used to encode states are now shared across arm types, you will need to replace the ECM, MTM or PSM part by DVRK (e.g. `PSM_HOMING_POWERING` is now `DVRK_HOMING_POWERING`).
  * In C++ class, events are now defined in two different scopes, `EventTriggers` is now split into `ClutchEvents` and `MessageEvents`.
* New features:
  * A new base class has been introduced to share the code between all arms, now all arms are derived from `mtsIntuititiveResearchKitArm`.  The base class implements most of the homing procedure as well as most control modes.  There is still some code duplicated code in the derived classes that should be ported to the base class. 
  * All arms now feature four control modes:
    * Direct joint:  This mode checks current arm state to make sure it is powered.  Commands are sent to PID directly.
    * Direct cartesian: This mode checks current arm state to make sure is powered and tool (for ECM and PSM) is past cannula.  Inverse kinematics is computed and new joints goal sent to PID.
    * Joint goal: Same pre-condition as direct joit but the robLSPB trajectory generator is called to make sure the robot moves smoothly to goal.
    * Cartesian goal: Same pre-condition as direct cartesian but the inverse kinematic is calculated for current and goal positions then trajectory in sent as joint goal.   NOTE: interpolation is performed in joint space.
  * Two new events have been added:
    * `RobotState` with payload `std::string` with the `DVRK_xyz` value corresponding to the current state.
    * `GoalReached` with payload `bool` to indicate if the trajectory has been fully executed. 
  * The command `SetRobotControlState` can now accept `std::string` matching all possible state values using `DVRK_xyz`
* Bug fixes:
  * Commands using the inverse kinematics now check the result of `robManipulator::InverseKinematics`.
  * Inverse kinematics for the ECM has been fixed.
  * Examples now build properly with catkin/ROS, i.e. they can be found in ROS path

1.0.1 (2015-01-30)
==================

* Change log file created
