#include <sawIntuitiveResearchKit/mtsSocketClientPSM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsSocketClientPSM, mtsTaskPeriodic);

mtsSocketClientPSM::mtsSocketClientPSM(const std::string &componentName, const double periodInSeconds,
                                       const std::string &ip, const unsigned int port) :
    mtsSocketBasePSM(componentName, periodInSeconds, ip, port, false)
{
    this->StateTable.AddData(PositionCartesianCurrent   , "PositionCartesianCurrent");
    this->StateTable.AddData(JawPosition   , "JawPosition");

    mtsInterfaceProvided *interfaceProvided = AddInterfaceProvided("Robot");
    if(interfaceProvided) {
        interfaceProvided->AddCommandReadState(this->StateTable, PositionCartesianCurrent, "GetPositionCartesian");
        interfaceProvided->AddCommandReadState(this->StateTable, JawPosition, "GetJawPosition");

        interfaceProvided->AddCommandVoid(&mtsSocketClientPSM::Freeze,
                                          this, "Freeze");
        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::SetPositionCartesian,
                                           this , "SetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::SetJawPosition,
                                           this , "SetJawPosition");
        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::SetRobotControlState,
                                           this , "SetRobotControlState");
        interfaceProvided->AddCommandRead(&mtsSocketClientPSM::GetRobotControlState,
                                           this , "GetRobotControlState");
        interfaceProvided->AddEventWrite(ErrorEvents, "Error", std::string(""));
    }
}

void mtsSocketClientPSM::Configure(const std::string & CMN_UNUSED(fileName))
{
    DesiredState = socketMessages::SCK_UNINITIALIZED;
    CurrentState = socketMessages::SCK_UNINITIALIZED;
    Command.Data.Header.Size = CLIENT_MSG_SIZE;
    Command.Socket->SetDestination(IpAddress, Command.IpPort);
    State.Socket->AssignPort(State.IpPort);
}

void mtsSocketClientPSM::Run()
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    ReceivePSMStateData();
    UpdateStatistics();
    SendPSMCommandData();    
}

void mtsSocketClientPSM::UpdateApplication()
{
    CurrentState = State.Data.RobotControlState;
    PositionCartesianCurrent.Valid() = (CurrentState == socketMessages::SCK_CART_POS);
    PositionCartesianCurrent.Position().FromNormalized(State.Data.CurrentPose);
    JawPosition = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::Freeze(void)
{
    DesiredState = socketMessages::SCK_CART_POS;
    Command.Data.GoalPose.From(State.Data.CurrentPose);
    Command.Data.GoalJaw = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::SetRobotControlState(const std::string &state)
{
    mtsIntuitiveResearchKitArmTypes::RobotStateType enumState = mtsIntuitiveResearchKitArmTypes::RobotStateTypeFromString(state);
    switch (enumState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED:
        DesiredState = socketMessages::SCK_UNINITIALIZED;
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER:
        case mtsIntuitiveResearchKitArmTypes::DVRK_READY:
        DesiredState = socketMessages::SCK_HOMED;
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN:
        DesiredState = socketMessages::SCK_CART_POS;
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN:
        DesiredState = socketMessages::SCK_CART_TRAJ;
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT:
        DesiredState = socketMessages::SCK_JNT_POS;
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT:
        DesiredState = socketMessages::SCK_JNT_TRAJ;
        break;
    default:
        std::cerr << CMN_LOG_DETAILS << state << " state not supported." << std::endl;
        break;
    }

    Command.Data.RobotControlState = DesiredState;
    Command.Data.GoalPose.From(State.Data.CurrentPose);
    Command.Data.GoalJaw = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::SetPositionCartesian(const prmPositionCartesianSet &position)
{
    if (DesiredState == socketMessages::SCK_CART_POS) {
        Command.Data.GoalPose.From(position.Goal());
    }
}

void mtsSocketClientPSM::SetJawPosition(const double &position)
{
    if (DesiredState == socketMessages::SCK_CART_POS) {
        Command.Data.GoalJaw = position;
    }
}

void mtsSocketClientPSM::GetRobotControlState(std::string &state) const
{
    mtsIntuitiveResearchKitArmTypes::RobotStateType enumState;
    switch (CurrentState) {
    case socketMessages::SCK_UNINITIALIZED:
        enumState = mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED;
        break;
    case socketMessages::SCK_HOMING:
        enumState = mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER;
        break;
    case socketMessages::SCK_HOMED:
        enumState = mtsIntuitiveResearchKitArmTypes::DVRK_READY;
        break;
    case socketMessages::SCK_CART_POS:
        enumState = mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN;
        break;
    case socketMessages::SCK_CART_TRAJ:
        enumState = mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN;
        break;
    case socketMessages::SCK_JNT_POS:
        enumState = mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT;
        break;
    case socketMessages::SCK_JNT_TRAJ:
        enumState = mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT;
        break;
    default:
        std::cerr << CMN_LOG_DETAILS << state << " state not supported." << std::endl;
        break;
    }

    state = mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(enumState);
}

void mtsSocketClientPSM::ReceivePSMStateData()
{
    // Recv Scoket Data
    size_t bytesRead = 0;
    bytesRead = State.Socket->Receive(State.Buffer, BUFFER_SIZE, TIMEOUT);
    if (bytesRead > 0) {
        if (bytesRead != State.Data.Header.Size){
            std::cerr << "Incorrect bytes read " << bytesRead << ". Looking for " << State.Data.Header.Size << " bytes." << std::endl;
        }

        std::stringstream ss;
        cmnDataFormat local, remote;
        ss.write(State.Buffer, bytesRead);

        // Dequeue all the datagrams and only use the latest one.
        int readCounter = 0;
        int dataLeft = bytesRead;
        while (dataLeft > 0) {
            dataLeft = State.Socket->Receive(State.Buffer, BUFFER_SIZE, 0);
            if (dataLeft != 0) {
                bytesRead = dataLeft;
            }

            readCounter++;
        }

        if (readCounter > 1)
            std::cerr << "Catching up : " << readCounter << std::endl;

        ss.write(State.Buffer, bytesRead);
        cmnData<socketStatePSM>::DeSerializeBinary(State.Data, ss, local, remote);

        State.Data.CurrentPose.NormalizedSelf();
        UpdateApplication();
    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "RecvPSMStateData: UDP receive failed" << std::endl;
    }
}

void mtsSocketClientPSM::SendPSMCommandData()
{
    // Update Header
    Command.Data.Header.Id++;
    Command.Data.Header.Timestamp = mTimeServer.GetRelativeTime();
    Command.Data.Header.LastId = State.Data.Header.Id;
    Command.Data.Header.LastTimestamp = State.Data.Header.Timestamp;
    Command.Data.RobotControlState = DesiredState;

    // Send Socket Data
    std::stringstream ss;
    cmnData<socketCommandPSM>::SerializeBinary(Command.Data, ss);
    memcpy(Command.Buffer, ss.str().c_str(), ss.str().length());

    Command.Socket->Send(Command.Buffer, ss.str().size());
}
