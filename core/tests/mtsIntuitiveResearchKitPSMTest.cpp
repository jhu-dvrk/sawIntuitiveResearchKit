// Regression coverage for independent snake joints and the physical IK/FK chain.
#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <fstream>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <sawIntuitiveResearchKit/robManipulatorPSMSnake.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>

class SnakePSMTestArm: public mtsIntuitiveResearchKitPSM {
public:
    SnakePSMTestArm(const std::string & model = "400141"): mtsIntuitiveResearchKitPSM("snake-test", 0.001)
    {
        m_snake_like = true;
        m_has_coupling = false;
        m_operating_state.SetState(prmOperatingState::ENABLED);
        m_control_space = mtsIntuitiveResearchKitControlTypes::JOINT_SPACE;
        m_control_mode = mtsIntuitiveResearchKitControlTypes::POSITION_MODE;
        auto snake = new robManipulatorPSMSnake;
        Manipulator = snake;
        Json::Value arm, tool;
        Json::Reader reader;
        std::ifstream arm_file(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR) + (model.substr(0, 3) == "420" ? "/kinematic/PSM_Si.json" : "/kinematic/PSM.json"));
        std::ifstream tool_file(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR) + "/tool/" + (model.substr(3) == "141" ? "ROUND_TIP_SCISSORS_" : model.substr(3) == "117" ? "NEEDLE_DRIVER_" : "MARYLAND_DISSECTOR_") + model + ".json");
        CPPUNIT_ASSERT(reader.parse(arm_file, arm));
        CPPUNIT_ASSERT(reader.parse(tool_file, tool));
        CPPUNIT_ASSERT_EQUAL(robManipulator::ESUCCESS, snake->LoadRobot(arm["DH"]));
        snake->Truncate(3);
        CPPUNIT_ASSERT_EQUAL(robManipulator::ESUCCESS, snake->LoadRobot(tool["DH"]));
        m_tool_present = m_tool_configured = true;
        update_configuration_js();
        m_jaw_configuration_js.PositionMin().SetSize(1, -1.0);
        m_jaw_configuration_js.PositionMax().SetSize(1, 1.0);
        m_pid_measured_js.Name().resize(7);
        m_pid_setpoint_js.Name().resize(7);
        m_pid_measured_js.Position().SetSize(7);
        m_pid_measured_js.Velocity().SetSize(7);
        m_pid_measured_js.Effort().SetSize(7);
        m_pid_setpoint_js.Position().SetSize(7);
        m_pid_setpoint_js.Velocity().SetSize(7);
        m_pid_setpoint_js.Effort().SetSize(7);
        for (size_t i = 0; i < 7; ++i) {
            m_pid_measured_js.Position().at(i) = .01 * (i + 1);
            m_pid_measured_js.Velocity().at(i) = .02 * (i + 1);
            m_pid_measured_js.Effort().at(i) = .03 * (i + 1);
            m_pid_setpoint_js.Position().at(i) = .04 * (i + 1);
            m_pid_setpoint_js.Velocity().at(i) = .05 * (i + 1);
            m_pid_setpoint_js.Effort().at(i) = .06 * (i + 1);
        }
        m_servo_jp.ForceAssign(m_pid_setpoint_js.Position());
        m_servo_jv.ForceAssign(m_pid_setpoint_js.Velocity());
        EnterToolEngaged();
        m_jaw_servo_jp = .28;
        m_trajectory_j.goal.ForceAssign(m_pid_setpoint_js.Position());
        m_trajectory_j.v.SetAll(1.0);
        m_trajectory_j.a.SetAll(2.0);
        m_trajectory_j.goal_v.Zeros();
        StateTable.PeriodStats.SetFromExisting(.001, 0, .001, .001, 0, 0, 0, 0, 1, 0, 1);
    }
    bool is_cartesian_ready(void) const override { return true; }
    bool is_joint_ready(void) const override { return true; }
    bool is_safe_for_cartesian_control(void) const override { return true; }
    void apply_feed_forward(void) override {}

    using mtsIntuitiveResearchKitPSM::UpdateStateJointKinematics;
    using mtsIntuitiveResearchKitPSM::servo_jp_internal;
    using mtsIntuitiveResearchKitPSM::jaw_move_jp;
    using mtsIntuitiveResearchKitArm::servo_jp;
    using mtsIntuitiveResearchKitArm::servo_jr;
    using mtsIntuitiveResearchKitArm::move_jp;
    using mtsIntuitiveResearchKitArm::control_servo_jp;
    using mtsIntuitiveResearchKitArm::control_move_jp;
    using mtsIntuitiveResearchKitArm::m_pid_measured_js;
    using mtsIntuitiveResearchKitArm::m_pid_setpoint_js;
    using mtsIntuitiveResearchKitArm::m_kin_measured_js;
    using mtsIntuitiveResearchKitArm::m_kin_setpoint_js;
    using mtsIntuitiveResearchKitArm::m_configuration_js;
    using mtsIntuitiveResearchKitArm::m_servo_jp_param;
    using mtsIntuitiveResearchKitArm::m_trajectory_j;
    using mtsIntuitiveResearchKitArm::m_control_space;
    using mtsIntuitiveResearchKitArm::m_control_mode;
    using mtsIntuitiveResearchKitPSM::m_jaw_setpoint_js;
    using mtsIntuitiveResearchKitArm::Manipulator;
};

class mtsIntuitiveResearchKitPSMTest: public CppUnit::TestFixture {
    CPPUNIT_TEST_SUITE(mtsIntuitiveResearchKitPSMTest);
    CPPUNIT_TEST(TestSixJointState);
    CPPUNIT_TEST(TestServoAndTrajectory);
    CPPUNIT_TEST(TestKinematicsBoundary);
    CPPUNIT_TEST_SUITE_END();
public:
    void TestSixJointState()
    {
        SnakePSMTestArm arm;
        arm.m_pid_setpoint_js.SetTimestamp(12.0);
        arm.m_pid_setpoint_js.SetValid(false);
        arm.UpdateStateJointKinematics();
        CPPUNIT_ASSERT_EQUAL(size_t(6), arm.m_configuration_js.Name().size());
        CPPUNIT_ASSERT_EQUAL(std::string("wrist_pitch"), arm.m_configuration_js.Name().at(4));
        CPPUNIT_ASSERT_EQUAL(std::string("wrist_yaw"), arm.m_configuration_js.Name().at(5));
        CPPUNIT_ASSERT_DOUBLES_EQUAL(.7853982, arm.m_configuration_js.PositionMax().at(4), 1e-10);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(.1, arm.m_configuration_js.EffortMax().at(4), 1e-10);
        CPPUNIT_ASSERT(arm.m_kin_measured_js.Position().Equal(arm.m_pid_measured_js.Position().Ref(6)));
        CPPUNIT_ASSERT(arm.m_kin_measured_js.Velocity().Equal(arm.m_pid_measured_js.Velocity().Ref(6)));
        CPPUNIT_ASSERT(arm.m_kin_measured_js.Effort().Equal(arm.m_pid_measured_js.Effort().Ref(6)));
        CPPUNIT_ASSERT(arm.m_kin_setpoint_js.Position().Equal(arm.m_pid_setpoint_js.Position().Ref(6)));
        CPPUNIT_ASSERT(arm.m_kin_setpoint_js.Velocity().Equal(arm.m_pid_setpoint_js.Velocity().Ref(6)));
        CPPUNIT_ASSERT(arm.m_kin_setpoint_js.Effort().Equal(arm.m_pid_setpoint_js.Effort().Ref(6)));
        CPPUNIT_ASSERT(!arm.m_jaw_setpoint_js.Valid());
        CPPUNIT_ASSERT_EQUAL(12.0, arm.m_kin_setpoint_js.Timestamp());
    }

    void TestServoAndTrajectory()
    {
        SnakePSMTestArm arm;
        const vctDoubleVec pid(arm.m_pid_setpoint_js.Position());
        // This seven-value input from the trajectory previously triggered Ref(8).
        arm.servo_jp_internal(pid, arm.m_pid_setpoint_js.Velocity());
        CPPUNIT_ASSERT(arm.m_servo_jp_param.Goal().Equal(pid));
        for (size_t i = 0; i < 6; ++i) {
            CPPUNIT_ASSERT_DOUBLES_EQUAL(arm.m_pid_setpoint_js.Velocity().at(i), arm.m_servo_jp_param.Velocity().at(i), 1e-12);
        }
        arm.UpdateStateJointKinematics();
        prmPositionJointSet command;
        command.Goal().ForceAssign(arm.m_kin_setpoint_js.Position());
        command.Velocity().ForceAssign(arm.m_kin_setpoint_js.Velocity());
        arm.servo_jp(command);
        arm.control_servo_jp();
        CPPUNIT_ASSERT(arm.m_servo_jp_param.Goal().Equal(pid));
        command.Velocity().SetSize(0);
        arm.servo_jp(command);
        arm.control_servo_jp();
        CPPUNIT_ASSERT_EQUAL(size_t(0), arm.m_servo_jp_param.Velocity().size());
        command.Goal().Zeros();
        command.Goal().at(4) = .01;
        arm.servo_jr(command);
        arm.control_servo_jp();
        CPPUNIT_ASSERT_DOUBLES_EQUAL(.21, arm.m_servo_jp_param.Goal().at(4), 1e-12);
        arm.move_jp(command);
        arm.control_move_jp();
        CPPUNIT_ASSERT((arm.m_trajectory_j.Reflexxes.ResultValue() == robReflexxes::Reflexxes_WORKING)
                       || (arm.m_trajectory_j.Reflexxes.ResultValue() == robReflexxes::Reflexxes_FINAL_STATE_REACHED));
        arm.m_control_space = mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE;
        arm.m_control_mode = mtsIntuitiveResearchKitControlTypes::POSITION_MODE;
        prmPositionJointSet jaw;
        jaw.Goal().SetSize(1, .4);
        arm.jaw_move_jp(jaw);
        CPPUNIT_ASSERT_EQUAL(size_t(7), arm.m_trajectory_j.goal.size());
    }

    void TestKinematicsBoundary()
    {
        for (const std::string model : {"400141", "420141", "400117", "420117", "400143"}) {
            SnakePSMTestArm arm(model);
            auto snake = dynamic_cast<robManipulatorPSMSnake *>(arm.Manipulator);
            vctDoubleVec q(6, 0.0);
            q.at(0) = .2; q.at(1) = -.1; q.at(2) = .1;
            q.at(3) = .3; q.at(4) = .2; q.at(5) = -.15;
            const auto full = robManipulatorPSMSnake::FullJoints(q);
            CPPUNIT_ASSERT(robManipulatorPSMSnake::ActiveJoints(full).Equal(q));
            const auto target = snake->robManipulator::ForwardKinematics(full);
            CPPUNIT_ASSERT((target - snake->ForwardKinematics(q)).Norm() < 1e-12);
            CPPUNIT_ASSERT((target - snake->ForwardKinematics(q, 6)).Norm() < 1e-12);
            vctDoubleMat jacobian;
            CPPUNIT_ASSERT(snake->JacobianBodyActive(q, jacobian));
            CPPUNIT_ASSERT_EQUAL(size_t(6), jacobian.cols());
            // Finite differences test the half-angle Jacobian chain rule.
            const double step = 1e-6;
            for (size_t col = 0; col < 6; ++col) {
                auto plus = q, minus = q;
                plus.at(col) += step; minus.at(col) -= step;
                const auto fp = snake->ForwardKinematics(plus);
                const auto fm = snake->ForwardKinematics(minus);
                const vct3 linear = target.Rotation().Transpose() * ((fp.Translation() - fm.Translation()) / (2 * step));
                const vctDouble3x3 angular = target.Rotation().Transpose() * ((fp.Rotation() - fm.Rotation()) / (2 * step));
                for (size_t row = 0; row < 3; ++row) {
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(linear.at(row), jacobian.at(row, col), 1e-6);
                }
                CPPUNIT_ASSERT_DOUBLES_EQUAL(angular.at(2, 1), jacobian.at(3, col), 1e-6);
                CPPUNIT_ASSERT_DOUBLES_EQUAL(angular.at(0, 2), jacobian.at(4, col), 1e-6);
                CPPUNIT_ASSERT_DOUBLES_EQUAL(angular.at(1, 0), jacobian.at(5, col), 1e-6);
            }
            auto seed = q;
            seed.at(4) += .01;
            CPPUNIT_ASSERT_EQUAL(robManipulator::ESUCCESS, snake->InverseKinematics(seed, target));
            CPPUNIT_ASSERT_EQUAL(size_t(6), seed.size());
            CPPUNIT_ASSERT((target - snake->ForwardKinematics(seed)).Norm() < 1e-7);
        }
    }
};
CPPUNIT_TEST_SUITE_REGISTRATION(mtsIntuitiveResearchKitPSMTest);
