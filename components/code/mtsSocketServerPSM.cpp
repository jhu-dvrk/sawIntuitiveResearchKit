#include <sawIntuitiveResearchKit/mtsSocketServerPSM.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsSocketServerPSM, mtsTaskPeriodic);

mtsSocketServerPSM::mtsSocketServerPSM(const std::string &componentName, const double periodInSeconds,
                                       const std::string &ip, const unsigned int port) :
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Command.Socket = new osaSocket(osaSocket::UDP);
    Command.Port = port;

    State.Socket = new osaSocket(osaSocket::UDP);
    State.Port = port+1;
    ClientIp = ip;

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
    State.Socket->SetDestination(ClientIp, State.Port);
    Command.Socket->AssignPort(Command.Port);
}

void mtsSocketServerPSM::Startup()
{
    State.Data.RobotControlState = 0;
}

void mtsSocketServerPSM::Cleanup()
{
    State.Socket->Close();
    Command.Socket->Close();
}

void mtsSocketServerPSM::Run()
{
    //State.Data.Error = "";
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    ReceivePSMCommandData();

    UpdatePSMState();
    SendPSMStateData();
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
        PositionCartesianSet.Goal().FromNormalized(Command.Data.GoalPose);
        SetPositionCartesian(PositionCartesianSet);

        SetJawPosition(Command.Data.GoalJaw);
        break;
    default:
        break;
    }
}

void mtsSocketServerPSM::UpdatePSMState()
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
}

void mtsSocketServerPSM::ReceivePSMCommandData()
{
    // Recv Socket Data
    size_t bytesRead = 0;
    bytesRead = Command.Socket->Receive(Command.Buffer, BUFFER_SIZE, 10.0*cmn_ms);
    if(bytesRead > 0){
        cmnDataFormat local, remote;
        std::stringstream ss;
        ss << Command.Buffer;
        cmnData<socketCommandPSM>::DeSerializeText(Command.Data, ss);
        ExecutePSMCommands();
    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "RecvPSMCommandData: UDP receive failed" << std::endl;
    }

}

void mtsSocketServerPSM::SendPSMStateData()
{
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
