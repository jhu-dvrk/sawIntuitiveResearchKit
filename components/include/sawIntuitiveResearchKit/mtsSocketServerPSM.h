#ifndef _mtsSocketServerPSM_h
#define _mtsSocketServerPSM_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstOSAbstraction/osaSocket.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>
#include <sawIntuitiveResearchKit/socketCommandPSM.h>
#include <sawIntuitiveResearchKit/socketStatePSM.h>

#define BUFFER_SIZE 1024

class mtsSocketServerPSM : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public :
    mtsSocketServerPSM(const std::string & componentName, const double periodInSeconds,
                       const std::string & ip, const unsigned int port);
    mtsSocketServerPSM(const mtsTaskPeriodicConstructorArg & arg);

    void Configure(const std::string &fileName = "");
    void Startup();
    void Run();
    void Cleanup();


protected:
    void ExecutePSMCommands();
    void UpdatePSMState();
    void ReceivePSMCommandData();
    void SendPSMStateData();
    void ErrorEventHandler(const std::string & message);

private:
    // UDP details
    struct {
        socketCommandPSM Data;
        osaSocket *Socket;
        short Port;
        char Buffer[BUFFER_SIZE];
    } Command;

    struct {
        socketStatePSM Data;
        osaSocket *Socket;
        short Port;
        char Buffer[BUFFER_SIZE];
    } State;

    std::string ClientIp;

    mtsFunctionWrite SetPositionCartesian;
    mtsFunctionWrite SetRobotControlState;
    mtsFunctionWrite SetJawPosition;
    mtsFunctionRead GetPositionCartesian;
    mtsFunctionRead GetRobotControlState;

    prmPositionCartesianGet PositionCartesianCurrent;
    prmPositionCartesianSet PositionCartesianSet;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSocketServerPSM);

#endif
