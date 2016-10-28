#include <sawIntuitiveResearchKit/mtsSocketClientPSM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsSocketClientPSM, mtsTaskPeriodic);

mtsSocketClientPSM::mtsSocketClientPSM(const std::string &componentName, const double periodInSeconds,
                                       const std::string &ip, const unsigned int port) :
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Command.Socket = new osaSocket(osaSocket::UDP);
    Command.Port = port;
    ServerIp = ip;

    State.Socket = new osaSocket(osaSocket::UDP);
    State.Port = port+1;

    this->StateTable.AddData(PositionCartesianCurrent   , "PositionCartesianCurrent");
    this->StateTable.AddData(JawPosition   , "JawPosition");

    mtsInterfaceProvided *interfaceProvided = AddInterfaceProvided("Robot");
    if(interfaceProvided) {
        interfaceProvided->AddCommandReadState(this->StateTable, PositionCartesianCurrent, "GetPositionCartesian");
        interfaceProvided->AddCommandReadState(this->StateTable, JawPosition, "GetJawPosition");
        interfaceProvided->AddCommandReadState(this->StateTable, StateTable.PeriodStats, "GetPeriodStatistics");

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
    Command.Socket->SetDestination(ServerIp, Command.Port);
    State.Socket->AssignPort(State.Port);
}

void mtsSocketClientPSM::Startup()
{
    Command.Data.RobotControlState = 0;
}

void mtsSocketClientPSM::Cleanup()
{
    Command.Socket->Close();
    State.Socket->Close();
}

void mtsSocketClientPSM::Run()
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    ReceivePSMStateData();

    SendPSMCommandData();
}

void mtsSocketClientPSM::UpdateApplication()
{
//    if(!State.Data.Error.empty())
//        ErrorEvents(mtsStdString(State.Data.Error));

    PositionCartesianCurrent.Valid() = (State.Data.RobotControlState == 1);
    PositionCartesianCurrent.Position().FromNormalized(State.Data.CurrentPose);
    JawPosition = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::Freeze(void)
{
    Command.Data.RobotControlState = 1;
    Command.Data.GoalPose.From(State.Data.CurrentPose);
    Command.Data.GoalJaw = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::SetRobotControlState(const std::string &state)
{
    mtsIntuitiveResearchKitArmTypes::RobotStateType
            dvrkState = mtsIntuitiveResearchKitArmTypes::RobotStateTypeFromString(state);

    // Check if the data is valid and is ok to send
    switch (dvrkState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN:
        Command.Data.RobotControlState = 1;
        break;
    default:
        Command.Data.RobotControlState = 0;
        break;
    }

    Command.Data.GoalPose.From(State.Data.CurrentPose);
    Command.Data.GoalJaw = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::SetPositionCartesian(const prmPositionCartesianSet &position)
{
    if (Command.Data.RobotControlState == 1) {
        Command.Data.GoalPose.From(position.Goal());
    }
}

void mtsSocketClientPSM::SetJawPosition(const double &position)
{
    if (Command.Data.RobotControlState == 1) {
        Command.Data.GoalJaw = position;
    }
}

void mtsSocketClientPSM::GetRobotControlState(std::string &state) const
{
    switch (State.Data.RobotControlState) {
    case 1:
        state = mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN);
        break;
    default:
        state = mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_READY);
        break;
    }
}

void mtsSocketClientPSM::ReceivePSMStateData()
{
    // Recv Scoket Data
    size_t bytesRead = 0;
    bytesRead = State.Socket->Receive(State.Buffer, BUFFER_SIZE, 10.0*cmn_ms);
    if (bytesRead > 0) {
        std::stringstream ss;
        ss << State.Buffer;
        cmnData<socketStatePSM>::DeSerializeText(State.Data, ss);
        State.Data.CurrentPose.NormalizedSelf();
        UpdateApplication();
    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "RecvPSMStateData: UDP receive failed" << std::endl;
    }
}

void mtsSocketClientPSM::SendPSMCommandData()
{
    // Send Socket Data
    std::stringstream ss;
    cmnData<socketCommandPSM>::SerializeText(Command.Data, ss);
    strcpy(Command.Buffer, ss.str().c_str());

    Command.Socket->Send(Command.Buffer, ss.str().size());
}
