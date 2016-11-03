#ifndef _mtsSocketClientPSM_h
#define _mtsSocketClientPSM_h

#include <sawIntuitiveResearchKit/mtsSocketBasePSM.h>

class mtsSocketClientPSM : public mtsSocketBasePSM
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public :
    mtsSocketClientPSM(const std::string & componentName, const double periodInSeconds,
                       const std::string & ip, const unsigned int port);
    mtsSocketClientPSM(const mtsTaskPeriodicConstructorArg & arg);

    void Configure(const std::string & fileName = "");    
    void Run();    

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
    prmPositionCartesianGet PositionCartesianCurrent;

    double JawPosition;
    mtsFunctionWrite ErrorEvents;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSocketClientPSM);

#endif
