#include <sawIntuitiveResearchKit/mtsSocketServerPSM.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsSocketServerPSM, mtsTaskPeriodic);

mtsSocketServerPSM::mtsSocketServerPSM(const std::string &componentName, const double periodInSeconds,
                                       const std::string &ip, const unsigned int port) :
    mtsSocketBasePSM(componentName, periodInSeconds, ip, port, true),
    mIsHoming(false)
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
    State.Data.Header.Size = SERVER_MSG_SIZE;
    State.Socket->SetDestination(IpAddress, State.IpPort);
    Command.Socket->AssignPort(Command.IpPort);
}

void mtsSocketServerPSM::Run()
{
    //State.Data.Error = "";
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    ReceivePSMCommandData();
    UpdateStatistics();
    SendPSMStateData();    
}

void mtsSocketServerPSM::ExecutePSMCommands()
{
    if(State.Data.RobotControlState != Command.Data.RobotControlState){
        switch (Command.Data.RobotControlState) {
        case 1:
            if(!mIsHoming){
                SetRobotControlState( mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER));
                mIsHoming = true;
            }
            break;
        case 2:
            SetRobotControlState( mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN));
            break;
        default:
            SetRobotControlState( mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_READY));
        }
    }

    switch (State.Data.RobotControlState) {
    case 2:
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
    bytesRead = Command.Socket->Receive(Command.Buffer, BUFFER_SIZE, TIMEOUT);
    if (bytesRead > 0) {
        if(bytesRead != Command.Data.Header.Size){
            std::cerr << "Incorrect bytes read " << bytesRead << ". Looking for " << Command.Data.Header.Size << " bytes." << std::endl;
        }                

        std::stringstream ss;
        cmnDataFormat local, remote;
        ss.write(Command.Buffer, bytesRead);

        // Dequeue all the datagrams and only use the latest one.
        int readCounter = 0;
        int dataLeft = bytesRead;
        while(dataLeft > 0){
            dataLeft = Command.Socket->Receive(Command.Buffer, BUFFER_SIZE, 0);
            if (dataLeft != 0) {
                bytesRead = dataLeft;
            }

            readCounter++;
        }

        if(readCounter > 1)
            std::cerr << "Catching up : " << readCounter << std::endl;

        ss.write(Command.Buffer, bytesRead);
        cmnData<socketCommandPSM>::DeSerializeBinary(Command.Data, ss, local, remote);

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
    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER :
        State.Data.RobotControlState = 1;
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN :
        State.Data.RobotControlState = 2;
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_READY :
        State.Data.RobotControlState = 3;
        mIsHoming = false;
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
    cmnData<socketStatePSM>::SerializeBinary(State.Data, ss);
    memcpy(State.Buffer, ss.str().c_str(), ss.str().length());

    State.Socket->Send(State.Buffer, ss.str().size());
}

void mtsSocketServerPSM::ErrorEventHandler(const std::string & message)
{
    // Send error message to the client
    //State.Data.Error = message;
    State.Data.RobotControlState = 0;
}
