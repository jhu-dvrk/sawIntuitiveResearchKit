#ifndef _mtsSocketClientPSM_h
#define _mtsSocketClientPSM_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstOSAbstraction/osaSocket.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>
#include <sawIntuitiveResearchKit/socketCommandPSM.h>
#include <sawIntuitiveResearchKit/socketStatePSM.h>

#define BUFFER_SIZE 1024

class mtsSocketClientPSM : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public :
    mtsSocketClientPSM(const std::string & componentName, const double periodInSeconds,
                       const std::string & ip, const unsigned int port);
    mtsSocketClientPSM(const mtsTaskPeriodicConstructorArg & arg);

    void Configure(const std::string & fileName = "");
    void Startup();
    void Run();
    void Cleanup();


protected:
    void SetRobotControlState(const std::string &state);
    void GetRobotControlState(std::string &state) const;

    void Freeze(void);
    void SetPositionCartesian(const prmPositionCartesianSet &position);
    void SetJawPosition(const double &position);
    void UpdateApplication();
    void ReceivePSMStateData();
    void SendPSMCommandData();

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

    std::string ServerIp;
    prmPositionCartesianGet PositionCartesianCurrent;

    double JawPosition;
    mtsFunctionWrite ErrorEvents;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSocketClientPSM);

#endif
