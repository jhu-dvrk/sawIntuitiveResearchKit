Change log
==========

1.2.0 (2015-10-18)
==================

* API changes:
  * Fixed digital input bug for state pressed.  Need to regenerate XML IO configuration files!
  * New PID has improved velosity estimation.  Need to use new PID XML files or at least update user files!
* Deprecated features:
  * None
* New features:
  * Console class can now be configured using json file for arms/teleops
  * Single program can be used for different console configurations, including for Qt widgets
  * Preliminary support for SUJ (using alpha version of dSIB)
  * Added support for base frame and commands to retrieve cartesian wrt arm base or base frame (for SUJ)
* Bug fixes:
  * Matlab config generator has been updated to create proper digital input "pressed" values
  * Matlab config generator forces to pick arm type, was generating improper joint directions on MTML
  * Use new velocity estimation, PID gains are now independant of periodicity
  * Fixed bugs in close/open grippers
  * PID configuration files now use joint name matching ROS/urdf files (in dvrk-ros)
  * PSM inverse kinematics now enforces tool can't go back in cannula

1.1.1 (2015-05-15)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * None
* Bug fixes:
  * Removed duplicate method Configure in MTM (base class Arm method is used)
  * PSM: increased pots to encoder tolerance during homing
  * Gripper calibration example: added code to preload encoders to avoid encoder bit overflow

1.1.0 (2015-04-28)
==================

* API changes:
  * SI units (meters) instead of millimeters are now used for all translation joints (ECM and PSM joint 3).  Cartesian positions were always based on SI.  `.rob` files were already meter based.  Users need to pay attention to:
    * PID gains.  We provided new PID configurations files for all arms but if you have old tuned gains for your system, you will need to increase them.
    * ROS joint topics.  If you used the ROS bridges, all joints were reported in millimeters.  MORE IMPORTANTLY, all joint commands for PSM and ECM translations were assumed to be in millimeters.  A move by 1 meant 1 millimeter.  It now means 1 meter!
  * Programmers using distances in the code should use the `cmn_mm` constant (e.g. `double tolerance = 1.0 * cmn_mm;`)
  * When building sawIntuitiveResearchKit, you must have cisst configured with `CISST_USE_SI_UNITS`.
  * Joint 7 on PSM is now referred to as "jaw", e.g. `SetPositionJaw` instead of `angle` or `gripper`.  On MTM, the last joint is still called "gripper"
  * For all derived classes (ECM, MTM and PSM), the enum types used to encode states are now shared across arm types, you will need to replace the ECM, MTM or PSM prefix by DVRK (e.g. `PSM_HOMING_POWERING` is now `DVRK_HOMING_POWERING`).
  * In C++ implementation, events are now defined in different scopes, struct `EventTriggers` is now split into `ClutchEvents`, `GripperEvents` and `MessageEvents`.
* Deprecated features:
  * None
* New features:
  * A new base class has been introduced to share code between arm types: `mtsIntuititiveResearchKitArm`.  The base class implements most of the homing procedure as well as most control modes.  There is still some code duplicated in the derived classes that should be ported to the base class, mostly related to state management.  We are considering the Boost MSM class for future revisions.
  * All arms now feature four control modes:
    * Joint direct: Arm must be ready.  Commands are sent to PID directly.
    * Cartesian direct: Arm must be ready and tool (for ECM and PSM) is past cannula.  Inverse kinematics is computed and new joints goal sent to PID.
    * Joint goal: Same pre-condition as direct joint but the `robLSPB` trajectory generator is called to make sure the robot moves smoothly to goal.  An event is sent when trajectory ends.
    * Cartesian goal: Same pre-condition as direct cartesian but the inverse kinematic is calculated for current and goal positions then trajectory in sent as joint goal.   NOTE: interpolation is performed in joint space.
  * Added commands to retrieve measured vs. desired positions, both for joints and cartesian.
  * Two new events have been added:
    * `RobotState` with payload `std::string` with the `DVRK_xyz` value corresponding to the current state.
    * `GoalReached` with payload `bool` to indicate if the trajectory has been fully executed.
  * The command `SetRobotControlState` can now accept `std::string` matching all possible state values using `DVRK_xyz`
  * The base class for all arms now has event handlers for PID errors (tracking and limits).  If any error occurs, trajectories using LSPB are stopped and an event `GoalReached(false)` is sent to caller.
  * All components now have three events for health status:
    * `Error`, `Warning` and `Status`.  All events have a `std::string` payload.
    * Most components used in the dVRK have been updated to act on error events (change to idle or safe state).  There is no automatic recovery, i.e. the user has to initiate the recovery using GUI and/or ROS interfaces.
    * Errors currently handled:
       * IO level: loss of power, watchdog timeout, firewire errors, pots to encoders discrepencies, encoder bit overflow.
       * PID: tracking errors.  Commands outside joint limits are treated as warning (values are clamped and an event with vector of booleans is sent to user).  When receiving error event, disable PID.
       * Arm: when receiving error event, goes to idle mode.  Need to re-home to use arm.
       * TeleOp: When error is caught or if positions are invalid (`IsValid()`), disables tele-op.
  * Console class is now connected to all IO, PID, arms and teleop components:
    * Catch error/warning and status messages from all components.
    * `Idle` button in Qt Widget allows to turn off all arms in software.
    * `TeleOp` button in Qt Widget allows to turn on all tele-op pairs.
  * Firewire communication now uses write broadcast by default when all FPGA-QLA boards have firmware 4.0 or higher.  For N boards, reduces packets from N reads + N writes to N reads and 1 write.  See `sawRobotIO1394` for details.
  * Matlab XML config file generator now adds a `-foot-pedal` suffix to the generated config file so it's easier to see which controller is configured for the foot pedals.
  * ROS interfaces have been updated to take advantage of new events and commands (see dvrk-ros repository).  Bridges can now be created in a more systematic way using the `dvrk_utilities` with the global functions `dvrk::add_mtm_topics`, ...
  * Tele-op:
    * Now handles error events and doesn't use cartesian positions for master/slave if marked as invalid
    * Updated GUI with messages, labels for master/slave, enable check button based on component state
* Bug fixes:
  * Commands using the inverse kinematics now check the result of `robManipulator::InverseKinematics`.
  * Inverse kinematics for the ECM has been fixed.
  * Examples now build properly with catkin/ROS, i.e. they can be found in ROS path
  * For MTM, when calling inverse kinematics with "current" joint position (initial guess for optimization), set joint 5 to 0 instead of joint 3 to pi/4 to try to move the wrist platform away from the user's hand.  The idea is to use the joint on top of the L bracket to move the L close to 90 deg off gripper.
  * For MTM, inverse kinematics uses closest modulus 2 pi position for last joint.
  * Encoders are now pre-loaded when the IO component starts.
  * IO level now checks encoder bit overflow.

1.0.1 (2015-01-30)
==================

* No change log file, initial release.
