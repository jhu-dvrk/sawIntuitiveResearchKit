#ifndef _MTSSOCKETBASEPSM_H
#define _MTSSOCKETBASEPSM_H

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstOSAbstraction/osaSocket.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>
#include <sawIntuitiveResearchKit/socketCommandPSM.h>
#include <sawIntuitiveResearchKit/socketStatePSM.h>

#define BUFFER_SIZE 1024

#define TIMEOUT 3000.0 * cmn_ms

class mtsSocketBasePSM : public mtsTaskPeriodic
{

public:
    mtsSocketBasePSM(const std::string & componentName, const double periodInSeconds,
                     const std::string & ip, const unsigned int port,
                     bool isServer);
    ~mtsSocketBasePSM() {}

    void Startup(){}
    void Cleanup();
    void UpdateStatistics();

protected:
    // UDP details
    struct {
        socketCommandPSM Data;
        osaSocket *Socket;
        short IpPort;
        char Buffer[BUFFER_SIZE];
    } Command;

    struct {
        socketStatePSM Data;
        osaSocket *Socket;
        short IpPort;
        char Buffer[BUFFER_SIZE];
    } State;

    std::string IpAddress;
    bool mIsServer;
    const osaTimeServer &mTimeServer;
private:
    unsigned int mPacketsLost;
    unsigned int mPacketsDelayed;
    double mLoopTime;
};


#endif // _MTSSOCKETBASEPSM_H
