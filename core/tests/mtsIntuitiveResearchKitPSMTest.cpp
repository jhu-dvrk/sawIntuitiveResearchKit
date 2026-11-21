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
        Json::Value arm, tool;
        Json::Reader reader;
        std::ifstream arm_file(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR) + (model.substr(0, 3) == "420" ? "/kinematic/PSM_Si.json" : "/kinematic/PSM.json"));
        std::ifstream tool_file(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR) + "/tool/" + (model.substr(3) == "141" ? "ROUND_TIP_SCISSORS_" : model.substr(3) == "117" ? "NEEDLE_DRIVER_" : "MARYLAND_DISSECTOR_") + model + ".json");
        CPPUNIT_ASSERT(reader.parse(arm_file, arm));
        CPPUNIT_ASSERT(reader.parse(tool_file, tool));
        const auto armConfig = loadJointConfig(arm["DH"]);
        const auto toolConfig = loadJointConfig(tool["DH"]);
        const auto armChain = loadKDLChain(arm["DH"]);
        const auto toolChain = loadKDLChain(tool["DH"]);
        CPPUNIT_ASSERT(armConfig && toolConfig && armChain && toolChain);
        arm_joints = *armConfig;
        arm_chain = *armChain;
        KDL::Chain fullChain = *armChain;
        fullChain.addChain(*toolChain);
        const auto fullConfig = armConfig->append(*toolConfig);
        FullManipulator = std::make_unique<robManipulatorGeneric>(fullConfig, fullChain);
        Manipulator = std::make_unique<robManipulatorPSMSnake>(fullConfig, fullChain);
        m_tool_present = m_tool_configured = true;
        update_configuration_js(Manipulator->jointConfig());
        m_jaw_configuration_js.PositionMin() = Eigen::ArrayXd::Constant(1, -1.0);
        m_jaw_configuration_js.PositionMax() = Eigen::ArrayXd::Constant(1, 1.0);
        m_pid_measured_js.Name().resize(7);
        m_pid_setpoint_js.Name().resize(7);
        m_pid_measured_js.Position().resize(7);
        m_pid_measured_js.Velocity().resize(7);
        m_pid_measured_js.Effort().resize(7);
        m_pid_setpoint_js.Position().resize(7);
        m_pid_setpoint_js.Velocity().resize(7);
        m_pid_setpoint_js.Effort().resize(7);
        for (size_t i = 0; i < 7; ++i) {
            m_pid_measured_js.Position()[i] = .01 * (i + 1);
            m_pid_measured_js.Velocity()[i] = .02 * (i + 1);
            m_pid_measured_js.Effort()[i] = .03 * (i + 1);
            m_pid_setpoint_js.Position()[i] = .04 * (i + 1);
            m_pid_setpoint_js.Velocity()[i] = .05 * (i + 1);
            m_pid_setpoint_js.Effort()[i] = .06 * (i + 1);
        }
        m_servo_jp = m_pid_setpoint_js.Position();
        m_servo_jv = m_pid_setpoint_js.Velocity();
        EnterToolEngaged();
        m_jaw_servo_jp = .28;
        m_trajectory_j.goal = m_pid_setpoint_js.Position();
        m_trajectory_j.v.setConstant(1.0);
        m_trajectory_j.a.setConstant(2.0);
        m_trajectory_j.goal_v.setZero();
        StateTable.PeriodStats.SetFromExisting(.001, 0, .001, .001, 0, 0, 0, 0, 1, 0, std::chrono::seconds(1));
    }
    std::unique_ptr<robManipulatorGeneric> FullManipulator;
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
        CPPUNIT_ASSERT_DOUBLES_EQUAL(.7853982, arm.m_configuration_js.PositionMax()[4], 1e-10);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(.1, arm.m_configuration_js.EffortMax()[4], 1e-10);
        CPPUNIT_ASSERT(arm.m_kin_measured_js.Position().isApprox(arm.m_pid_measured_js.Position().head(6)));
        CPPUNIT_ASSERT(arm.m_kin_measured_js.Velocity().isApprox(arm.m_pid_measured_js.Velocity().head(6)));
        CPPUNIT_ASSERT(arm.m_kin_measured_js.Effort().isApprox(arm.m_pid_measured_js.Effort().head(6)));
        CPPUNIT_ASSERT(arm.m_kin_setpoint_js.Position().isApprox(arm.m_pid_setpoint_js.Position().head(6)));
        CPPUNIT_ASSERT(arm.m_kin_setpoint_js.Velocity().isApprox(arm.m_pid_setpoint_js.Velocity().head(6)));
        CPPUNIT_ASSERT(arm.m_kin_setpoint_js.Effort().isApprox(arm.m_pid_setpoint_js.Effort().head(6)));
        CPPUNIT_ASSERT(!arm.m_jaw_setpoint_js.Valid());
        CPPUNIT_ASSERT_EQUAL(12.0, arm.m_kin_setpoint_js.Timestamp());
    }

    void TestServoAndTrajectory()
    {
        SnakePSMTestArm arm;
        const Eigen::VectorXd pid(arm.m_pid_setpoint_js.Position());
        // This seven-value input from the trajectory previously triggered Ref(8).
        arm.servo_jp_internal(pid, arm.m_pid_setpoint_js.Velocity());
        CPPUNIT_ASSERT(arm.m_servo_jp_param.Goal().isApprox(pid));
        for (size_t i = 0; i < 6; ++i) {
            CPPUNIT_ASSERT_DOUBLES_EQUAL(arm.m_pid_setpoint_js.Velocity()[i], arm.m_servo_jp_param.Velocity()[i], 1e-12);
        }
        arm.UpdateStateJointKinematics();
        prmPositionJointSet command;
        command.Goal() = arm.m_kin_setpoint_js.Position();
        command.Velocity() = arm.m_kin_setpoint_js.Velocity();
        arm.servo_jp(command);
        arm.control_servo_jp();
        CPPUNIT_ASSERT(arm.m_servo_jp_param.Goal().isApprox(pid));
        command.Velocity().resize(0);
        arm.servo_jp(command);
        arm.control_servo_jp();
        CPPUNIT_ASSERT_EQUAL(Eigen::Index(0), arm.m_servo_jp_param.Velocity().size());
        command.Goal().setZero();
        command.Goal()[4] = .01;
        arm.servo_jr(command);
        arm.control_servo_jp();
        CPPUNIT_ASSERT_DOUBLES_EQUAL(.21, arm.m_servo_jp_param.Goal()[4], 1e-12);
        arm.move_jp(command);
        arm.control_move_jp();
        CPPUNIT_ASSERT((arm.m_trajectory_j.Reflexxes.ResultValue() == robReflexxes::ResultType::WORKING)
                       || (arm.m_trajectory_j.Reflexxes.ResultValue() == robReflexxes::ResultType::FINAL_STATE_REACHED));
        arm.m_control_space = mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE;
        arm.m_control_mode = mtsIntuitiveResearchKitControlTypes::POSITION_MODE;
        prmPositionJointSet jaw;
        jaw.Goal() = Eigen::VectorXd::Constant(1, .4);
        arm.jaw_move_jp(jaw);
        CPPUNIT_ASSERT_EQUAL(Eigen::Index(7), arm.m_trajectory_j.goal.size());
    }

    void TestKinematicsBoundary()
    {
        for (const std::string model : {"400141", "420141", "400117", "420117", "400143"}) {
            SnakePSMTestArm arm(model);
            auto snake = dynamic_cast<robManipulatorPSMSnake *>(arm.Manipulator.get());
            Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
            q[0] = .2; q[1] = -.1; q[2] = .1;
            q[3] = .3; q[4] = .2; q[5] = -.15;
            const auto full = robManipulatorPSMSnake::FullJoints(q);
            CPPUNIT_ASSERT(robManipulatorPSMSnake::ActiveJoints(full).isApprox(q));
            const auto target = arm.FullManipulator->forwardKinematics(full);
            CPPUNIT_ASSERT((target.matrix() - snake->forwardKinematics(q).matrix()).norm() < 1e-12);
            CPPUNIT_ASSERT((target.matrix() - snake->forwardKinematics(q, 6).matrix()).norm() < 1e-12);
            snake->computeJacobians(q);
            const Eigen::MatrixXd jacobian = snake->bodyJacobian().matrix();
            CPPUNIT_ASSERT_EQUAL(Eigen::Index(6), jacobian.cols());
            // Finite differences test the half-angle Jacobian chain rule.
            const double step = 1e-6;
            for (size_t col = 0; col < 6; ++col) {
                auto plus = q, minus = q;
                plus[col] += step; minus[col] -= step;
                const auto fp = snake->forwardKinematics(plus);
                const auto fm = snake->forwardKinematics(minus);
                const Eigen::Vector3d linear = target.linear().transpose() * ((fp.translation() - fm.translation()) / (2 * step));
                const Eigen::Matrix3d angular = target.linear().transpose() * ((fp.linear() - fm.linear()) / (2 * step));
                for (size_t row = 0; row < 3; ++row) {
                    CPPUNIT_ASSERT_DOUBLES_EQUAL(linear[row], jacobian(row, col), 1e-6);
                }
                CPPUNIT_ASSERT_DOUBLES_EQUAL(angular(2, 1), jacobian(3, col), 1e-6);
                CPPUNIT_ASSERT_DOUBLES_EQUAL(angular(0, 2), jacobian(4, col), 1e-6);
                CPPUNIT_ASSERT_DOUBLES_EQUAL(angular(1, 0), jacobian(5, col), 1e-6);
            }
            auto seed = q;
            seed[4] += .01;
            std::string error;
            CPPUNIT_ASSERT_MESSAGE(error, snake->inverseKinematics(target, seed, error));
            CPPUNIT_ASSERT_EQUAL(Eigen::Index(6), seed.size());
            CPPUNIT_ASSERT((target.matrix() - snake->forwardKinematics(seed).matrix()).norm() < 1e-7);
        }
    }
};
CPPUNIT_TEST_SUITE_REGISTRATION(mtsIntuitiveResearchKitPSMTest);
