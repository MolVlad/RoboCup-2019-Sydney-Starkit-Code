#include "SPQR_GC_Connector.h"

#include <unistd.h>
#include <iostream>
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"




SPQR_GC_Connector::SPQR_GC_Connector(){

    SPQR::ConfigurationParameters();
    udp = new UdpComm();
    //udpS = new UdpComm();
    if(!udp->setBlocking(false) ||
         !udp->setBroadcast(true) ||
         !udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) ||
         !udp->setLoopback(false))
      {
        fprintf(stderr, "libgamectrl: Could not open UDP port\n");
        delete udp;
        udp = 0;
        // continue, because button interface will still work
      }
    setted = false;
    }


bool SPQR_GC_Connector::receive(RoboCup::RoboCupGameControlData *buff_ptr, UdpComm *udp){
    bool received = false;
    int size;
    //RoboCup::RoboCupGameControlData buffer = *buff_ptr;
    
    struct sockaddr_in from;
    socklen_t fromLen = sizeof(from);
    size = ::recvfrom(udp->getSock(), (char*) (buff_ptr), sizeof(*buff_ptr), 0, (sockaddr*) &from, &fromLen);
    char * ip = inet_ntoa(from.sin_addr);
    if(size != -1 && buffer.header[0]=='R'){
        if(setted == false){
         udp->setTarget(ip, GAMECONTROLLER_RETURN_PORT);
         setted = true;
        }
        
         received = true;
/*
              std::cout << "####################################\n\n[EMA] GC_header: " << static_cast<char*>(buffer.header) << std::endl;
      std::cout << "[EMA] GC_version: " << static_cast<int>(buffer.version) << std::endl;
      std::cout << "[EMA] GC_packetNumber: " << static_cast<int>(buffer.packetNumber) << std::endl;
      std::cout << "[EMA] GC_playersPerTeam: " << static_cast<int>(buffer.playersPerTeam) << std::endl;
      std::cout << "[EMA] GC_competitionPhase: " << static_cast<int>(buffer.competitionPhase) << std::endl;
      std::cout << "[EMA] GC_competitionType: " << static_cast<int>(buffer.competitionType) << std::endl;
      std::cout << "[EMA] GC_gamePhase: " << static_cast<int>(buffer.gamePhase) << std::endl;
      std::cout << "[EMA] GC_state: " << static_cast<int>(buffer.state) << std::endl;
      std::cout << "[EMA] GC_setPlay: " << static_cast<int>(buffer.setPlay) << std::endl;
      std::cout << "[EMA] GC_firstHalf: " << static_cast<int>(buffer.firstHalf) << std::endl;
      std::cout << "[EMA] GC_kickingTeam: " << static_cast<int>(buffer.kickingTeam) << std::endl;
      std::cout << "[EMA] GC_secsRemaining: " << static_cast<int>(buffer.secsRemaining) << std::endl;
      std::cout << "[EMA] GC_secondaryTime: " << static_cast<int>(buffer.secondaryTime) << std::endl;
      std::cout << "[EMA] GC_Team[0].teamNumber: " << static_cast<int>(buffer.teams[0].teamNumber) << std::endl;
      std::cout << "[EMA] GC_Team[0].score: " << static_cast<int>(buffer.teams[0].score) << std::endl;
      std::cout << "[EMA] GC_Team[0].players[IO].penalty: " << static_cast<int>(buffer.teams[0].players[theRobotInfo.number-1].penalty) << std::endl;
      std::cout << "\n\n####################################\n\n";
*/
    }

    return received;
}

bool SPQR_GC_Connector::send(uint8_t message, UdpComm * udp, uint8_t teamNumber, uint8_t playerNumber)
  {
    RoboCup::RoboCupGameControlReturnData returnPacket;
    
    returnPacket.team = (uint8_t) teamNumber;
    returnPacket.player = (uint8_t) playerNumber;
    returnPacket.message = message;
    return !udp || udp->write((const char*) &returnPacket, sizeof(returnPacket));
  }


//TODO Implementare divisione in griglia tramite mixture di Gaussiane e copertura con quadrati di grandezza variabile in base a questo.
//TODO per Robocup

/*


*/

 std::vector<TeamInfoGC> SPQR_GC_Connector::fillTeamInfo(RoboCup::TeamInfo tmf[]){
  std::vector<TeamInfoGC> teams = std::vector<TeamInfoGC>(2);
  teams.at(0).players = std::vector<RobotInfoGC>(6);
  teams.at(1).players = std::vector<RobotInfoGC>(6);
      
  for(unsigned int i = 0; i < teams.size(); i++){
    teams.at(i).teamNumber = tmf[i].teamNumber;
    teams.at(i).teamColour = tmf[i].teamColor;
    teams.at(i).score = tmf[i].score;
    teams.at(i).penaltyShot = tmf[i].penaltyShot;
    teams.at(i).singleShots = tmf[i].singleShots;
    for(unsigned int k = 0; k < teams.at(i).players.size(); k++){
      teams.at(i).players.at(k).penalty = tmf[i].players[k].penalty;
      teams.at(i).players.at(k).secsTillUnpenalised = tmf[i].players[k].secsTillUnpenalised; 
    }
  }
  return teams;

}

void SPQR_GC_Connector::update(GCData& gcd) {
  //TODO CHECK HEADER
    //RoboCup::RoboCupGameControlData *buff_ptr;
    //buff_ptr = &buffer;
    auto now = std::chrono::system_clock::now();
    int teamNumber = Global::getSettings().teamNumber;
    
    if(receive(&buffer, udp)){
      // Fill all Variables of Our Representation GCData, thx to GC packets
      gcd.version = static_cast<int>(buffer.version);
      gcd.packetNumber = static_cast<int>(buffer.packetNumber);
      gcd.playersPerTeam = static_cast<int>(buffer.playersPerTeam);
      gcd.competitionPhase = static_cast<int>(buffer.competitionPhase);
      gcd.competitionType = static_cast<int>(buffer.competitionType);
      gcd.gamePhase = static_cast<int>(buffer.gamePhase);
      gcd.state = static_cast<int>(buffer.state);
      gcd.setPlay = static_cast<int>(buffer.setPlay);
      gcd.firstHalf = static_cast<int>(buffer.firstHalf);
      gcd.kickingTeam = static_cast<int>(buffer.kickingTeam);
      gcd.secsRemaining = static_cast<int>(buffer.secsRemaining);
      gcd.secondaryTime = static_cast<int>(buffer.secondaryTime);

      int robotNumber = theRobotInfo.number; // Robot Number
      int teamIndex = (static_cast<int>(buffer.teams[0].teamNumber) == teamNumber) ? 0 : 1; // Check Team Number, index because from GC we receive an array of TeamInfo dim 2
      gcd.teams = fillTeamInfo(buffer.teams);
      
      //////////PART OF PREVIOUS STATES //////////////
      if(gcd.state != gcd.previousState ||
          gcd.gamePhase != gcd.previousGamePhase ||
          gcd.kickingTeam != gcd.previousKickingTeam ||
          gcd.teams.at(teamIndex).teamColour != gcd.previousTeamColour ||
          gcd.teams.at(teamIndex).players[robotNumber-1].penalty != gcd.previousPenalty){

        gcd.previousState = gcd.state;
        gcd.previousGamePhase = gcd.gamePhase;
        gcd.previousKickingTeam = gcd.kickingTeam;
        gcd.previousTeamColour = gcd.teams.at(teamIndex).teamColour;
        gcd.previousPenalty =gcd.teams.at(teamIndex).players[robotNumber-1].penalty;
      }


      
      bool ret = false;
    
      ret = send(GAMECONTROLLER_RETURN_MSG_ALIVE, udp, static_cast<uint8_t>(teamNumber), static_cast<uint8_t>(theRobotInfo.number));
      
    }
    return;
}

MAKE_MODULE(SPQR_GC_Connector, motionInfrastructure)
