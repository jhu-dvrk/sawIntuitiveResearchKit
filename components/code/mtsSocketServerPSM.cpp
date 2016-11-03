#include <sawIntuitiveResearchKit/mtsSocketServerPSM.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsSocketServerPSM, mtsTaskPeriodic);

mtsSocketServerPSM::mtsSocketServerPSM(const std::string &componentName, const double periodInSeconds,
                                       const std::string &ip, const unsigned int port) :
    mtsSocketBasePSM(componentName, periodInSeconds, ip, port, true)
{
    mtsInterfaceRequired *interfaceRequired = AddInterfaceRequired("PSM");
    if(interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", GetPositionCartesian);
        interfaceRequired->AddFunction("SetPositionCartesian", SetPositionCartesian);
        interfaceRequired->AddFunction("SetJawPosition"      , SetJawPosition);
        interfaceRequired->AddFunction("GetRobotControlState", GetRobotControlState);
        interfaceRequired->AddFunction("SetRobotControlState", SetRobotControlState);
        interfaceRequired->AddEventHandlerWrite(&mtsSocketServerPSM::ErrorEventHandler,
                                                this, "Error");
    }    
}

void mtsSocketServerPSM::Configure(const std::string & CMN_UNUSED(fileName))
{
    State.Socket->SetDestination(IpAddress, State.IpPort);
    Command.Socket->AssignPort(Command.IpPort);
}

void mtsSocketServerPSM::Run()
{
    //State.Data.Error = "";
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    ReceivePSMCommandData();
    SendPSMStateData();
    UpdateStatistics();
}

void mtsSocketServerPSM::ExecutePSMCommands()
{
    if(State.Data.RobotControlState != Command.Data.RobotControlState){
        switch (Command.Data.RobotControlState) {
        case 1:
            SetRobotControlState( mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN));
            break;
        default:
            SetRobotControlState( mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_READY));
        }
    }

    switch (State.Data.RobotControlState) {
    case 1:
        PositionCartesianSet.Goal().From(Command.Data.GoalPose);
        SetPositionCartesian(PositionCartesianSet);

        SetJawPosition(Command.Data.GoalJaw);
        break;
    default:
        break;
    }
}

void mtsSocketServerPSM::ReceivePSMCommandData()
{
    // Recv Socket Data
    size_t bytesRead = 0;
    bytesRead = Command.Socket->Receive(Command.Buffer, BUFFER_SIZE, 10.0*cmn_ms);
    if (bytesRead > 0) {
        std::stringstream ss;
        ss << Command.Buffer;
        cmnData<socketCommandPSM>::DeSerializeText(Command.Data, ss);
        Command.Data.GoalPose.NormalizedSelf();
        ExecutePSMCommands();

    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "RecvPSMCommandData: UDP receive failed" << std::endl;
    }
}

void mtsSocketServerPSM::SendPSMStateData()
{
    // Update PSM State
    mtsExecutionResult executionResult;

    // Get Cartesian position
    executionResult = GetPositionCartesian(PositionCartesianCurrent);
    State.Data.CurrentPose.Assign(PositionCartesianCurrent.Position());

    // Get Robot State
    mtsStdString psmState;
    GetRobotControlState(psmState);

    mtsIntuitiveResearchKitArmTypes::RobotStateType state = mtsIntuitiveResearchKitArmTypes::RobotStateTypeFromString(psmState.Data);
    switch (state) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN :
        State.Data.RobotControlState = 1;
        break;
    default:
        State.Data.RobotControlState = 0;
        break;
    }

    // Update Header
    State.Data.Header.Id++;
    State.Data.Header.Timestamp = mTimeServer.GetRelativeTime();
    State.Data.Header.LastId = Command.Data.Header.Id;
    State.Data.Header.LastTimestamp = Command.Data.Header.Timestamp;

    // Send Socket Data
    std::stringstream ss;
    cmnData<socketStatePSM>::SerializeText(State.Data, ss);
    strcpy(State.Buffer, ss.str().c_str());

    State.Socket->Send(State.Buffer, ss.str().size());
}

void mtsSocketServerPSM::ErrorEventHandler(const std::string & message)
{
    // Send error message to the client
    //State.Data.Error = message;
    State.Data.RobotControlState = 0;
}
