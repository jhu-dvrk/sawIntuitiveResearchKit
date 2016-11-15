#ifndef _mtsSocketServerPSM_h
#define _mtsSocketServerPSM_h

#include <sawIntuitiveResearchKit/mtsSocketBasePSM.h>

class mtsSocketServerPSM : public mtsSocketBasePSM
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public :
    mtsSocketServerPSM(const std::string & componentName, const double periodInSeconds,
                       const std::string & ip, const unsigned int port);
    mtsSocketServerPSM(const mtsTaskPeriodicConstructorArg & arg);

    void Configure(const std::string &fileName = "");
    void Run();    

protected:
    void ExecutePSMCommands();
    void UpdatePSMState();
    void ReceivePSMCommandData();
    void SendPSMStateData();
    void ErrorEventHandler(const std::string & message);

private:
    mtsFunctionWrite SetPositionCartesian;
    mtsFunctionWrite SetRobotControlState;
    mtsFunctionWrite SetJawPosition;
    mtsFunctionRead GetPositionCartesian;
    mtsFunctionRead GetRobotControlState;

    prmPositionCartesianGet PositionCartesianCurrent;
    prmPositionCartesianSet PositionCartesianSet;
    bool mIsHoming;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSocketServerPSM);

#endif
