
#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"

#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/spqr_representations/GCData.h"
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wunknown-warning-option"
#pragma clang diagnostic ignored "-Wconversion"
#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wunused-local-typedef"
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#pragma clang diagnostic ignored "-fno-exceptions"

#endif
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <arpa/inet.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <assert.h>
#include <cstring>
#include <chrono>
#include <RoboCupGameControlData.h>
#include "libgamectrl/UdpComm.h"
#include "Representations/Infrastructure/RobotInfo.h"



#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

#include "Tools/Settings.h"

MODULE(SPQR_GC_Connector, 
{,
 USES(RobotInfo),
 PROVIDES(GCData),
});

class SPQR_GC_Connector : public SPQR_GC_ConnectorBase
{
private:
	RoboCup::RoboCupGameControlData buffer;   
	UdpComm *udp;
	UdpComm *udpS;
	bool receive(RoboCup::RoboCupGameControlData *buff_ptr, UdpComm *udp); 
	bool send(uint8_t message, UdpComm * udp, uint8_t teamNumber, uint8_t playerNumber);

public:
    
	std::vector<TeamInfoGC> fillTeamInfo(RoboCup::TeamInfo tmf[]);
    bool setted;
    void update(GCData& gcd);
    SPQR_GC_Connector();
};
