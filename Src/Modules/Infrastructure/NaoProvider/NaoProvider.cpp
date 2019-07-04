/**
 * @file Modules/Infrastructure/NaoProvider.cpp
 * The file declares a module that provides information from the Nao via DCM.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "NaoProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

MAKE_MODULE(NaoProvider, motionInfrastructure)

#ifdef TARGET_ROBOT

#include "Platform/Time.h"
#include "Platform/File.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Settings.h"

#include "libbhuman/bhuman.h"

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <sys/socket.h>
#include "../Util/Buildchain/gcc/include/msgpack.hpp"
#include <cstring>
#include "Tools/spqrSocketMethods/SPQRSocket2.h"
//#include "libgamectrl/gamecontroller/GameController.hpp"
#include <unistd.h>

static char socketname[14] = "/tmp/robocup\0";

RoboCup::RoboCupGameControlData NaoProvider::getGCDatas(){
  RoboCup::RoboCupGameControlData rgc;
  rgc.version = theGCData.version;
  rgc.packetNumber = theGCData.packetNumber;
  rgc.playersPerTeam = theGCData.playersPerTeam;
  rgc.competitionPhase = theGCData.competitionPhase;
  rgc.competitionType = theGCData.competitionType;
  rgc.gamePhase = theGCData.gamePhase;
  rgc.state = theGCData.state;
  rgc.setPlay = theGCData.setPlay;
  rgc.firstHalf = theGCData.firstHalf;
  rgc.kickingTeam = theGCData.kickingTeam;
  rgc.secsRemaining = theGCData.secsRemaining;
  rgc.secondaryTime = theGCData.secondaryTime;
  for(unsigned int i = 0; i < theGCData.teams.size(); i++){
    rgc.teams[i].teamNumber = theGCData.teams.at(i).teamNumber;
    rgc.teams[i].teamColor = theGCData.teams.at(i).teamColour;
    rgc.teams[i].score = theGCData.teams.at(i).score;
    rgc.teams[i].penaltyShot = theGCData.teams.at(i).penaltyShot;
    rgc.teams[i].singleShots = theGCData.teams.at(i).singleShots;
    for(unsigned int k = 0; k < theGCData.teams.at(i).players.size(); k++){
      rgc.teams[i].players[k].penalty = theGCData.teams.at(i).players.at(k).penalty;
      rgc.teams[i].players[k].secsTillUnpenalised = theGCData.teams.at(i).players.at(k).secsTillUnpenalised;
    }//k
  }//i
  return rgc;
}

void replaceAll2(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}

LolaRead unpackLolaData2( msgpack::object obj){
  LolaRead lrd;
  msgpack::object_kv* kv;
  int mapSize = obj.via.map.size;
  int i = 0;
  int k = 0;
  while(i < mapSize){
    kv = obj.via.map.ptr + i;
    std::stringstream nss;
    nss<<kv->key;
    std::string a = nss.str();
    
    k = 0;
    while(k < kv->val.via.array.size){
      std::stringstream nss2;
      nss2<<*(kv->val.via.array.ptr +k);
      std::string tp = nss2.str();
      replaceAll2(tp,"\"","");
      
      if(a == "\"RobotConfig\""){
        if(k == 0){
          
          strcpy(lrd.bodyId,tp.c_str());
        
        }else if(k == 1){
          if(tp == "\"6.0.0\""){
            
            lrd.bodyVersion = NAOVersion::V6;  
          }else{
            
            lrd.bodyVersion = NAOVersion::V6;
          }
          
        
        }else if(k ==2){
          strcpy(lrd.headId,tp.c_str());
        }else{
          if(tp == "\"6.0.0\""){
            
            lrd.headVersion = NAOVersion::V6;  
          }else{
            
            lrd.headVersion = NAOVersion::V6;
          }
        }

      }else if(a == "\"Accelerometer\""){
        lrd.accelerometer[k] = std::stof(tp);

      }else if(a == "\"Angles\""){
        lrd.angles[k] = std::stof(tp);

      }else if(a == "\"Battery\""){
        lrd.battery[k] = std::stof(tp);

      }else if(a == "\"Current\""){
        lrd.current[k] = std::stof(tp);

      }else if(a == "\"FSR\""){
        lrd.FSR[k] = std::stof(tp);
      }else if(a == "\"Gyroscope\""){
        lrd.gyroscope[k] = std::stof(tp);

      }else if(a == "\"Position\""){
        lrd.positions[k] = std::stof(tp);

      }else if(a == "\"Sonar\""){
        lrd.sonar[k] = std::stof(tp);

      }else if(a == "\"Stiffness\""){
        lrd.stiffness[k] = std::stof(tp);

      }else if(a == "\"Temperature\""){
        lrd.temperature[k] = std::stof(tp);

      }else if(a == "\"Touch\""){
        lrd.touch[k] = std::stof(tp);

      }else if(a == "\"Status\""){
        lrd.status[k] = std::stoi(tp);
      }
      k++;
    }
    i++;
  }
   return lrd; 
}


thread_local NaoProvider* NaoProvider::theInstance = nullptr;

NaoProvider::NaoProvider()
{
  NaoProvider::theInstance = this;
  memset(&gameControlData, 0, sizeof(gameControlData));
    
  if(Global::getSettings().headName == Global::getSettings().bodyName)
    OUTPUT_TEXT("Hi, I am " << Global::getSettings().headName << ".");
  else
    OUTPUT_TEXT("Hi, I am " << Global::getSettings().headName << " (using " << Global::getSettings().bodyName << "'s body).");

  OUTPUT(idRobotname, bin, Global::getSettings().headName << Global::getSettings().bodyName << Global::getSettings().location);
       
  int sockLola = make_named_socket2(socketname);
  std::cout<<"sockNumber"<<sockLola<<"\n";
  theInstance->socket = sockLola;
  
  
  for(int i = 0; i < Joints::numOfJoints; ++i)
    clippedLastFrame[i] = SensorData::off;
}

NaoProvider::~NaoProvider()
{
  NaoProvider::theInstance = nullptr;

}

void NaoProvider::finishFrame()
{
  if(theInstance)
    theInstance->send();
}

void NaoProvider::waitForFrameData()
{
  {
    const int size = 896; 
    char *buff = new char[size];
    int byteRead = 0;
    int ret = 0;
  
    ret = ricevi2(theInstance->socket, buff, size);
    
    std::stringstream ss;
    msgpack::unpacked result;
    unpack(result, buff,size);
    msgpack::object obj(result.get());
    ss << obj;
    theInstance->lrd =unpackLolaData2(obj);

  }
  

  if(theInstance){
  }
}

void NaoProvider::send()
{
 
  theInstance->lwr.positions.at(0) = theJointRequest.angles[Joints::headYaw];
  theInstance->lwr.stiffness.at(0) = 0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::headYaw];
  theInstance->lwr.positions.at(1) = theJointRequest.angles[Joints::headPitch];
  theInstance->lwr.stiffness.at(1) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::headPitch];
  theInstance->lwr.positions.at(2) = theJointRequest.angles[Joints::lShoulderPitch];
  theInstance->lwr.stiffness.at(2) = 0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lShoulderPitch];
  theInstance->lwr.positions.at(3) = theJointRequest.angles[Joints::lShoulderRoll];
  theInstance->lwr.stiffness.at(3) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lShoulderRoll];
  theInstance->lwr.positions.at(4) = theJointRequest.angles[Joints::lElbowYaw];
  theInstance->lwr.stiffness.at(4) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lElbowYaw];
  theInstance->lwr.positions.at(5) =  theJointRequest.angles[Joints::lElbowRoll];
  theInstance->lwr.stiffness.at(5) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lElbowRoll];
  theInstance->lwr.positions.at(6) =  theJointRequest.angles[Joints::lWristYaw];
  theInstance->lwr.stiffness.at(6) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lWristYaw];
  theInstance->lwr.positions.at(7) = theJointRequest.angles[Joints::lHipYawPitch];
  theInstance->lwr.stiffness.at(7) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch];
  theInstance->lwr.positions.at(8) = theJointRequest.angles[Joints::lHipRoll];
  theInstance->lwr.stiffness.at(8) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lHipRoll];
  theInstance->lwr.positions.at(9) =  theJointRequest.angles[Joints::lHipPitch];
  theInstance->lwr.stiffness.at(9) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lHipPitch];
  theInstance->lwr.positions.at(10) =  theJointRequest.angles[Joints::lKneePitch];
  theInstance->lwr.stiffness.at(10) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lKneePitch];
  theInstance->lwr.positions.at(11) =  theJointRequest.angles[Joints::lAnklePitch];
  theInstance->lwr.stiffness.at(11) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lAnklePitch];
  theInstance->lwr.positions.at(12) =  theJointRequest.angles[Joints::lAnkleRoll];
  theInstance->lwr.stiffness.at(12) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lAnkleRoll];
  theInstance->lwr.positions.at(13) = theJointRequest.angles[Joints::rHipRoll];
  theInstance->lwr.stiffness.at(13) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rHipRoll];
  theInstance->lwr.positions.at(14) =  theJointRequest.angles[Joints::rHipPitch];
  theInstance->lwr.stiffness.at(14) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rHipPitch];
  theInstance->lwr.positions.at(15) =  theJointRequest.angles[Joints::rKneePitch];
  theInstance->lwr.stiffness.at(15) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rKneePitch];
  theInstance->lwr.positions.at(16) = theJointRequest.angles[Joints::rAnklePitch];
  theInstance->lwr.stiffness.at(16) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rAnklePitch];
  theInstance->lwr.positions.at(17) =  theJointRequest.angles[Joints::rAnkleRoll];
  theInstance->lwr.stiffness.at(17) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rAnkleRoll];
  theInstance->lwr.positions.at(18) =  theJointRequest.angles[Joints::rShoulderPitch];
  theInstance->lwr.stiffness.at(18) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rShoulderPitch];
  theInstance->lwr.positions.at(19) =  theJointRequest.angles[Joints::rShoulderRoll];
  theInstance->lwr.stiffness.at(19) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rShoulderRoll];
  theInstance->lwr.positions.at(20) =  theJointRequest.angles[Joints::rElbowYaw];
  theInstance->lwr.stiffness.at(20) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rElbowYaw];
  theInstance->lwr.positions.at(21) =  theJointRequest.angles[Joints::rElbowRoll];
  theInstance->lwr.stiffness.at(21) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rElbowRoll];
  theInstance->lwr.positions.at(22) =  theJointRequest.angles[Joints::rWristYaw];
  theInstance->lwr.stiffness.at(22) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rWristYaw];
  theInstance->lwr.positions.at(23) =  theJointRequest.angles[Joints::lHand];
  theInstance->lwr.stiffness.at(23) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::lHand];
  theInstance->lwr.positions.at(24) =  theJointRequest.angles[Joints::rHand];
  theInstance->lwr.stiffness.at(24) =  0.01f*theJointRequest.stiffnessData.stiffnesses[Joints::rHand];

  theInstance->lwr.lear.at(0) = theLEDRequest.ledStates[LEDRequest::earsLeft0Deg];
  theInstance->lwr.lear.at(1) = theLEDRequest.ledStates[LEDRequest::earsLeft36Deg];
  theInstance->lwr.lear.at(2) = theLEDRequest.ledStates[LEDRequest::earsLeft72Deg];
  theInstance->lwr.lear.at(3) = theLEDRequest.ledStates[LEDRequest::earsLeft108Deg];
  theInstance->lwr.lear.at(4) = theLEDRequest.ledStates[LEDRequest::earsLeft144Deg];
  theInstance->lwr.lear.at(5) = theLEDRequest.ledStates[LEDRequest::earsLeft180Deg];
  theInstance->lwr.lear.at(6) = theLEDRequest.ledStates[LEDRequest::earsLeft216Deg];
  theInstance->lwr.lear.at(7) = theLEDRequest.ledStates[LEDRequest::earsLeft252Deg];
  theInstance->lwr.lear.at(8) = theLEDRequest.ledStates[LEDRequest::earsLeft288Deg];
  theInstance->lwr.lear.at(9) = theLEDRequest.ledStates[LEDRequest::earsLeft324Deg];
  theInstance->lwr.rear.at(0) = theLEDRequest.ledStates[LEDRequest::earsRight324Deg];
  theInstance->lwr.rear.at(1) = theLEDRequest.ledStates[LEDRequest::earsRight288Deg];
  theInstance->lwr.rear.at(2) = theLEDRequest.ledStates[LEDRequest::earsRight252Deg];
  theInstance->lwr.rear.at(3) = theLEDRequest.ledStates[LEDRequest::earsRight216Deg];
  theInstance->lwr.rear.at(4) = theLEDRequest.ledStates[LEDRequest::earsRight180Deg];
  theInstance->lwr.rear.at(5) = theLEDRequest.ledStates[LEDRequest::earsRight144Deg];
  theInstance->lwr.rear.at(6) = theLEDRequest.ledStates[LEDRequest::earsRight108Deg];
  theInstance->lwr.rear.at(7) = theLEDRequest.ledStates[LEDRequest::earsRight72Deg];
  theInstance->lwr.rear.at(8) = theLEDRequest.ledStates[LEDRequest::earsRight36Deg];
  theInstance->lwr.rear.at(9) = theLEDRequest.ledStates[LEDRequest::earsRight0Deg];
  theInstance->lwr.chest.at(0) = theLEDRequest.ledStates[LEDRequest::chestRed];
  theInstance->lwr.chest.at(1) = theLEDRequest.ledStates[LEDRequest::chestGreen];
  theInstance->lwr.chest.at(2) = theLEDRequest.ledStates[LEDRequest::chestBlue];
  theInstance->lwr.leye.at(0) = theLEDRequest.ledStates[LEDRequest::faceLeftRed45Deg];
  theInstance->lwr.leye.at(1) = theLEDRequest.ledStates[LEDRequest::faceLeftRed0Deg];
  theInstance->lwr.leye.at(2) = theLEDRequest.ledStates[LEDRequest::faceLeftRed315Deg];
  theInstance->lwr.leye.at(3) = theLEDRequest.ledStates[LEDRequest::faceLeftRed270Deg];
  theInstance->lwr.leye.at(4) = theLEDRequest.ledStates[LEDRequest::faceLeftRed225Deg];
  theInstance->lwr.leye.at(5) = theLEDRequest.ledStates[LEDRequest::faceLeftRed180Deg];
  theInstance->lwr.leye.at(6) = theLEDRequest.ledStates[LEDRequest::faceLeftRed135Deg];
  theInstance->lwr.leye.at(7) = theLEDRequest.ledStates[LEDRequest::faceLeftRed90Deg];
  theInstance->lwr.leye.at(8) = theLEDRequest.ledStates[LEDRequest::faceLeftGreen45Deg];
  theInstance->lwr.leye.at(9) = theLEDRequest.ledStates[LEDRequest::faceLeftGreen0Deg];
  theInstance->lwr.leye.at(10) = theLEDRequest.ledStates[LEDRequest::faceLeftGreen315Deg];
  theInstance->lwr.leye.at(11) = theLEDRequest.ledStates[LEDRequest::faceLeftGreen270Deg];
  theInstance->lwr.leye.at(12) = theLEDRequest.ledStates[LEDRequest::faceLeftGreen225Deg];
  theInstance->lwr.leye.at(13) = theLEDRequest.ledStates[LEDRequest::faceLeftGreen180Deg];
  theInstance->lwr.leye.at(14) = theLEDRequest.ledStates[LEDRequest::faceLeftGreen135Deg];
  theInstance->lwr.leye.at(15) = theLEDRequest.ledStates[LEDRequest::faceLeftGreen90Deg];
  theInstance->lwr.leye.at(16) = theLEDRequest.ledStates[LEDRequest::faceLeftBlue45Deg];
  theInstance->lwr.leye.at(17) = theLEDRequest.ledStates[LEDRequest::faceLeftBlue0Deg];
  theInstance->lwr.leye.at(18) = theLEDRequest.ledStates[LEDRequest::faceLeftBlue315Deg];
  theInstance->lwr.leye.at(19) = theLEDRequest.ledStates[LEDRequest::faceLeftBlue270Deg];
  theInstance->lwr.leye.at(20) = theLEDRequest.ledStates[LEDRequest::faceLeftBlue225Deg];
  theInstance->lwr.leye.at(21) = theLEDRequest.ledStates[LEDRequest::faceLeftBlue180Deg];
  theInstance->lwr.leye.at(22) = theLEDRequest.ledStates[LEDRequest::faceLeftBlue135Deg];
  theInstance->lwr.leye.at(23) = theLEDRequest.ledStates[LEDRequest::faceLeftBlue90Deg];
  theInstance->lwr.reye.at(0) = theLEDRequest.ledStates[LEDRequest::faceRightRed0Deg];
  theInstance->lwr.reye.at(1) = theLEDRequest.ledStates[LEDRequest::faceRightRed45Deg];
  theInstance->lwr.reye.at(2) = theLEDRequest.ledStates[LEDRequest::faceRightRed90Deg];
  theInstance->lwr.reye.at(3) = theLEDRequest.ledStates[LEDRequest::faceRightRed135Deg];
  theInstance->lwr.reye.at(4) = theLEDRequest.ledStates[LEDRequest::faceRightRed180Deg];
  theInstance->lwr.reye.at(5) = theLEDRequest.ledStates[LEDRequest::faceRightRed225Deg];
  theInstance->lwr.reye.at(6) = theLEDRequest.ledStates[LEDRequest::faceRightRed270Deg];
  theInstance->lwr.reye.at(7) = theLEDRequest.ledStates[LEDRequest::faceRightRed315Deg];
  theInstance->lwr.reye.at(8) = theLEDRequest.ledStates[LEDRequest::faceRightGreen0Deg];
  theInstance->lwr.reye.at(9) = theLEDRequest.ledStates[LEDRequest::faceRightGreen45Deg];
  theInstance->lwr.reye.at(10) = theLEDRequest.ledStates[LEDRequest::faceRightGreen90Deg];
  theInstance->lwr.reye.at(11) = theLEDRequest.ledStates[LEDRequest::faceRightGreen135Deg];
  theInstance->lwr.reye.at(12) = theLEDRequest.ledStates[LEDRequest::faceRightGreen180Deg];
  theInstance->lwr.reye.at(13) = theLEDRequest.ledStates[LEDRequest::faceRightGreen225Deg];
  theInstance->lwr.reye.at(14) = theLEDRequest.ledStates[LEDRequest::faceRightGreen270Deg];
  theInstance->lwr.reye.at(15) = theLEDRequest.ledStates[LEDRequest::faceRightGreen315Deg];
  theInstance->lwr.reye.at(16) = theLEDRequest.ledStates[LEDRequest::faceRightBlue0Deg];
  theInstance->lwr.reye.at(17) = theLEDRequest.ledStates[LEDRequest::faceRightBlue45Deg];
  theInstance->lwr.reye.at(18) = theLEDRequest.ledStates[LEDRequest::faceRightBlue90Deg];
  theInstance->lwr.reye.at(19) = theLEDRequest.ledStates[LEDRequest::faceRightBlue135Deg];
  theInstance->lwr.reye.at(20) = theLEDRequest.ledStates[LEDRequest::faceRightBlue180Deg];
  theInstance->lwr.reye.at(21) = theLEDRequest.ledStates[LEDRequest::faceRightBlue225Deg];
  theInstance->lwr.reye.at(22) = theLEDRequest.ledStates[LEDRequest::faceRightBlue270Deg];
  theInstance->lwr.reye.at(23) = theLEDRequest.ledStates[LEDRequest::faceRightBlue315Deg];
  theInstance->lwr.lfoot.at(0) = theLEDRequest.ledStates[LEDRequest::footLeftRed];
  theInstance->lwr.lfoot.at(1) = theLEDRequest.ledStates[LEDRequest::footLeftGreen];
  theInstance->lwr.lfoot.at(2) = theLEDRequest.ledStates[LEDRequest::footLeftBlue];
  theInstance->lwr.rfoot.at(0) = theLEDRequest.ledStates[LEDRequest::footRightRed];
  theInstance->lwr.rfoot.at(1) = theLEDRequest.ledStates[LEDRequest::footRightGreen];
  theInstance->lwr.rfoot.at(2) = theLEDRequest.ledStates[LEDRequest::footRightBlue];
  theInstance->lwr.skull.at(0) = theLEDRequest.ledStates[LEDRequest::headLedFrontLeft1];
  theInstance->lwr.skull.at(1) = theLEDRequest.ledStates[LEDRequest::headLedFrontLeft0];
  theInstance->lwr.skull.at(2) = theLEDRequest.ledStates[LEDRequest::headLedMiddleLeft0];
  theInstance->lwr.skull.at(3) = theLEDRequest.ledStates[LEDRequest::headLedRearLeft0];
  theInstance->lwr.skull.at(4) = theLEDRequest.ledStates[LEDRequest::headLedRearLeft1];
  theInstance->lwr.skull.at(5) = theLEDRequest.ledStates[LEDRequest::headLedRearLeft2];
  theInstance->lwr.skull.at(6) = theLEDRequest.ledStates[LEDRequest::headLedRearRight2];
  theInstance->lwr.skull.at(7) = theLEDRequest.ledStates[LEDRequest::headLedRearRight1];
  theInstance->lwr.skull.at(8) = theLEDRequest.ledStates[LEDRequest::headLedRearRight0];
  theInstance->lwr.skull.at(9) = theLEDRequest.ledStates[LEDRequest::headLedMiddleRight0];
  theInstance->lwr.skull.at(10) = theLEDRequest.ledStates[LEDRequest::headLedFrontRight0];
  theInstance->lwr.skull.at(11) = theLEDRequest.ledStates[LEDRequest::headLedFrontRight1];

  for(unsigned int i = 0; i < theGCData.teams.size(); i++){
      if(theGCData.teams.at(i).teamNumber == 19){
    	  for(unsigned int k = 0; k < theGCData.teams.at(i).players.size(); k++){
    		  	  	if(k == theRobotInfo.number){
    		  	  		if(theGCData.teams.at(i).players.at(k-1).penalty !=PENALTY_NONE){

    		  	  			theInstance->lwr.chest.at(0) = 1;
							theInstance->lwr.chest.at(1) = 0;
							theInstance->lwr.chest.at(2) = 0;
						}else{
							theInstance->lwr.chest.at(0) = 0;
							theInstance->lwr.chest.at(1) = 1;
							theInstance->lwr.chest.at(2) = 0;
						}
    		  	  	}


    	        }//k
      }

    }//i

std::map<std::string, std::vector<float>> v { {"Chest", theInstance->lwr.chest} , {"Position", theInstance->lwr.positions}, {"Stiffness", theInstance->lwr.stiffness}, {"REye", theInstance->lwr.reye}, {"LEye", theInstance->lwr.leye},
              {"LFoot", theInstance->lwr.lfoot}, {"RFoot", theInstance->lwr.rfoot}, {"Skull", theInstance->lwr.skull} };
  
  std::stringstream buffer;
  msgpack::pack(buffer, v);
  
   int byteRead = 0;
    int ret = 0;

  const char *c = buffer.str().c_str();
  invia2(theInstance->socket,c,(int)buffer.str().size());  
}

/*
* New update methods for LolA client
*/


void NaoProvider::update(FrameInfo& frameInfo)
{
  frameInfo.time = std::max(frameInfo.time + 1, Time::getCurrentSystemTime());

  if(gameControlData.packetNumber != naoBody.getGameControlData().packetNumber)
    gameControlTimeStamp = frameInfo.time;
  gameControlData = getGCDatas();
}

//overrided the sensors instance with lrd 
void NaoProvider::update(FsrSensorData& fsrSensorData)
{
  fsrSensorData.pressures[Legs::left][FsrSensors::fl] = theInstance->lrd.FSR[0];
  fsrSensorData.pressures[Legs::left][FsrSensors::fr] = theInstance->lrd.FSR[1];
  fsrSensorData.pressures[Legs::left][FsrSensors::bl] = theInstance->lrd.FSR[2];
  fsrSensorData.pressures[Legs::left][FsrSensors::br] = theInstance->lrd.FSR[3];
  fsrSensorData.pressures[Legs::right][FsrSensors::fl] = theInstance->lrd.FSR[4];
  fsrSensorData.pressures[Legs::right][FsrSensors::fr] = theInstance->lrd.FSR[5];
  fsrSensorData.pressures[Legs::right][FsrSensors::bl] = theInstance->lrd.FSR[6];
  fsrSensorData.pressures[Legs::right][FsrSensors::br] = theInstance->lrd.FSR[7];
  fsrSensorData.totals[Legs::left] = theInstance->lrd.FSR[0] + theInstance->lrd.FSR[1] +theInstance->lrd.FSR[2] +theInstance->lrd.FSR[3];
  fsrSensorData.totals[Legs::right] = theInstance->lrd.FSR[4] + theInstance->lrd.FSR[5] +theInstance->lrd.FSR[6] +theInstance->lrd.FSR[7];
}

//3 instances for each different field
void NaoProvider::update(InertialSensorData& inertialSensorData)
{
  
// TODO: verify signs
  inertialSensorData.gyro.x() = theInstance->lrd.gyroscope[0]*0.965f;
  inertialSensorData.gyro.y() = theInstance->lrd.gyroscope[1];
  //TODO CHECK AFTER CHINA
  inertialSensorData.gyro.z() = theInstance->lrd.gyroscope[2]*1.0764f; // Aldebarans z-gyron is negated for some reason...

  inertialSensorData.acc.x() = -theInstance->lrd.accelerometer[0];
  inertialSensorData.acc.y() = -theInstance->lrd.accelerometer[1];
  inertialSensorData.acc.z() = -theInstance->lrd.accelerometer[2];
  
  inertialSensorData.angle.x() = theInstance->lrd.angles[0];
  inertialSensorData.angle.y() = theInstance->lrd.angles[1];
  inertialSensorData.angle.z() = 0.f; // -sensors[angleZSensor]; //TODO CHECK VINCENZO
}


void NaoProvider::update(JointSensorData& jointSensorData)
{ 
  float* sensorsPos = theInstance->lrd.positions;//naoBody.getSensors();
  float* sensorsCurr = theInstance->lrd.current;
  float* sensorsTemp = theInstance->lrd.temperature;
  float* sensorsStat = theInstance->lrd.status;

  jointSensorData.angles[Joints::rHand] = sensorsPos[24] - theJointCalibration.offsets[Joints::rHand];
  jointSensorData.currents[Joints::rHand] = static_cast<short>(1000.f * sensorsCurr[24]);
  jointSensorData.temperatures[Joints::rHand] = static_cast<unsigned char>(sensorsTemp[24]);
  jointSensorData.status[Joints::rHand] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[24]));

  jointSensorData.angles[Joints::lHand] = sensorsPos[23] - theJointCalibration.offsets[Joints::lHand];
  jointSensorData.currents[Joints::lHand] = static_cast<short>(1000.f * sensorsCurr[23]);
  jointSensorData.temperatures[Joints::lHand] = static_cast<unsigned char>(sensorsTemp[23]);
  jointSensorData.status[Joints::lHand] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[23]));

  jointSensorData.angles[Joints::rWristYaw] = sensorsPos[22] - theJointCalibration.offsets[Joints::rWristYaw];
  jointSensorData.currents[Joints::rWristYaw] = static_cast<short>(1000.f * sensorsCurr[22]);
  jointSensorData.temperatures[Joints::rWristYaw] = static_cast<unsigned char>(sensorsTemp[22]);
  jointSensorData.status[Joints::rWristYaw] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[22]));

  jointSensorData.angles[Joints::rElbowRoll] = sensorsPos[21] - theJointCalibration.offsets[Joints::rElbowRoll];
  jointSensorData.currents[Joints::rElbowRoll] = static_cast<short>(1000.f * sensorsCurr[21]);
  jointSensorData.temperatures[Joints::rElbowRoll] = static_cast<unsigned char>(sensorsTemp[21]);
  jointSensorData.status[Joints::rElbowRoll] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[21]));

  jointSensorData.angles[Joints::rElbowYaw] = sensorsPos[20] - theJointCalibration.offsets[Joints::rElbowYaw];
  jointSensorData.currents[Joints::rElbowYaw] = static_cast<short>(1000.f * sensorsCurr[20]);
  jointSensorData.temperatures[Joints::rElbowYaw] = static_cast<unsigned char>(sensorsTemp[20]);
  jointSensorData.status[Joints::rElbowYaw] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[20]));

  jointSensorData.angles[Joints::rShoulderRoll] = sensorsPos[19] - theJointCalibration.offsets[Joints::rShoulderRoll];
  jointSensorData.currents[Joints::rShoulderRoll] = static_cast<short>(1000.f * sensorsCurr[19]);
  jointSensorData.temperatures[Joints::rShoulderRoll] = static_cast<unsigned char>(sensorsTemp[19]);
  jointSensorData.status[Joints::rShoulderRoll] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[19]));

  jointSensorData.angles[Joints::rShoulderPitch] = sensorsPos[18] - theJointCalibration.offsets[Joints::rShoulderPitch];
  jointSensorData.currents[Joints::rShoulderPitch] = static_cast<short>(1000.f * sensorsCurr[18]);
  jointSensorData.temperatures[Joints::rShoulderPitch] = static_cast<unsigned char>(sensorsTemp[18]);
  jointSensorData.status[Joints::rShoulderPitch] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[18]));

  jointSensorData.angles[Joints::rAnkleRoll] = sensorsPos[17] - theJointCalibration.offsets[Joints::rAnkleRoll];
  jointSensorData.currents[Joints::rAnkleRoll] = static_cast<short>(1000.f * sensorsCurr[17]);
  jointSensorData.temperatures[Joints::rAnkleRoll] = static_cast<unsigned char>(sensorsTemp[17]);
  jointSensorData.status[Joints::rAnkleRoll] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[17]));

  jointSensorData.angles[Joints::rAnklePitch] = sensorsPos[16] - theJointCalibration.offsets[Joints::rAnklePitch];
  jointSensorData.currents[Joints::rAnklePitch] = static_cast<short>(1000.f * sensorsCurr[16]);
  jointSensorData.temperatures[Joints::rAnklePitch] = static_cast<unsigned char>(sensorsTemp[16]);
  jointSensorData.status[Joints::rAnklePitch] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[16]));

  jointSensorData.angles[Joints::rKneePitch] = sensorsPos[15] - theJointCalibration.offsets[Joints::rKneePitch];
  jointSensorData.currents[Joints::rKneePitch] = static_cast<short>(1000.f * sensorsCurr[15]);
  jointSensorData.temperatures[Joints::rKneePitch] = static_cast<unsigned char>(sensorsTemp[15]);
  jointSensorData.status[Joints::rKneePitch] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[15]));

  jointSensorData.angles[Joints::rHipPitch] = sensorsPos[14] - theJointCalibration.offsets[Joints::rHipPitch];
  jointSensorData.currents[Joints::rHipPitch] = static_cast<short>(1000.f * sensorsCurr[14]);
  jointSensorData.temperatures[Joints::rHipPitch] = static_cast<unsigned char>(sensorsTemp[14]);
  jointSensorData.status[Joints::rHipPitch] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[14]));

  jointSensorData.angles[Joints::rHipRoll] = sensorsPos[13] - theJointCalibration.offsets[Joints::rHipRoll];
  jointSensorData.currents[Joints::rHipRoll] = static_cast<short>(1000.f * sensorsCurr[13]);
  jointSensorData.temperatures[Joints::rHipRoll] = static_cast<unsigned char>(sensorsTemp[13]);
  jointSensorData.status[Joints::rHipRoll] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[13]));

  jointSensorData.angles[Joints::lAnkleRoll] = sensorsPos[12] - theJointCalibration.offsets[Joints::lAnkleRoll];
  jointSensorData.currents[Joints::lAnkleRoll] = static_cast<short>(1000.f * sensorsCurr[12]);
  jointSensorData.temperatures[Joints::lAnkleRoll] = static_cast<unsigned char>(sensorsTemp[12]);
  jointSensorData.status[Joints::lAnkleRoll] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[12]));

  jointSensorData.angles[Joints::lAnklePitch] = sensorsPos[11] - theJointCalibration.offsets[Joints::lAnklePitch];
  jointSensorData.currents[Joints::lAnklePitch] = static_cast<short>(1000.f * sensorsCurr[11]);
  jointSensorData.temperatures[Joints::lAnklePitch] = static_cast<unsigned char>(sensorsTemp[11]);
  jointSensorData.status[Joints::lAnklePitch] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[11]));

  jointSensorData.angles[Joints::lKneePitch] = sensorsPos[10] - theJointCalibration.offsets[Joints::lKneePitch];
  jointSensorData.currents[Joints::lKneePitch] = static_cast<short>(1000.f * sensorsCurr[10]);
  jointSensorData.temperatures[Joints::lKneePitch] = static_cast<unsigned char>(sensorsTemp[10]);
  jointSensorData.status[Joints::lKneePitch] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[10]));

  jointSensorData.angles[Joints::lHipPitch] = sensorsPos[9] - theJointCalibration.offsets[Joints::lHipPitch];
  jointSensorData.currents[Joints::lHipPitch] = static_cast<short>(1000.f * sensorsCurr[9]);
  jointSensorData.temperatures[Joints::lHipPitch] = static_cast<unsigned char>(sensorsTemp[9]);
  jointSensorData.status[Joints::lHipPitch] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[9]));

  jointSensorData.angles[Joints::lHipRoll] = sensorsPos[8] - theJointCalibration.offsets[Joints::lHipRoll];
  jointSensorData.currents[Joints::lHipRoll] = static_cast<short>(1000.f * sensorsCurr[8]);
  jointSensorData.temperatures[Joints::lHipRoll] = static_cast<unsigned char>(sensorsTemp[8]);
  jointSensorData.status[Joints::lHipRoll] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[8]));

  // lHipYawPitch and virtualized rHipYawPitch
  jointSensorData.angles[Joints::lHipYawPitch] = sensorsPos[7] - theJointCalibration.offsets[Joints::lHipYawPitch];
  jointSensorData.currents[Joints::lHipYawPitch] = static_cast<short>(1000.f * sensorsCurr[7]);
  jointSensorData.temperatures[Joints::lHipYawPitch] = static_cast<unsigned char>(sensorsTemp[7]);
  jointSensorData.status[Joints::lHipYawPitch] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[7]));

  jointSensorData.angles[Joints::rHipYawPitch] = jointSensorData.angles[Joints::lHipYawPitch];
  jointSensorData.currents[Joints::rHipYawPitch] = jointSensorData.currents[Joints::lHipYawPitch];
  jointSensorData.temperatures[Joints::rHipYawPitch] = jointSensorData.temperatures[Joints::lHipYawPitch];
  jointSensorData.status[Joints::rHipYawPitch] = jointSensorData.status[Joints::lHipYawPitch];

  jointSensorData.angles[Joints::lWristYaw] = sensorsPos[6] - theJointCalibration.offsets[Joints::lWristYaw];
  jointSensorData.currents[Joints::lWristYaw] = static_cast<short>(1000.f * sensorsCurr[6]);
  jointSensorData.temperatures[Joints::lWristYaw] = static_cast<unsigned char>(sensorsTemp[6]);
  jointSensorData.status[Joints::lWristYaw] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[6]));

  jointSensorData.angles[Joints::lElbowRoll] = sensorsPos[5] - theJointCalibration.offsets[Joints::lElbowRoll];
  jointSensorData.currents[Joints::lElbowRoll] = static_cast<short>(1000.f * sensorsCurr[5]);
  jointSensorData.temperatures[Joints::lElbowRoll] = static_cast<unsigned char>(sensorsTemp[5]);
  jointSensorData.status[Joints::lElbowRoll] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[5]));

  jointSensorData.angles[Joints::lElbowYaw] = sensorsPos[4] - theJointCalibration.offsets[Joints::lElbowYaw];
  jointSensorData.currents[Joints::lElbowYaw] = static_cast<short>(1000.f * sensorsCurr[4]);
  jointSensorData.temperatures[Joints::lElbowYaw] = static_cast<unsigned char>(sensorsTemp[4]);
  jointSensorData.status[Joints::lElbowYaw] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[4]));

  jointSensorData.angles[Joints::lShoulderRoll] = sensorsPos[3] - theJointCalibration.offsets[Joints::lShoulderRoll];
  jointSensorData.currents[Joints::lShoulderRoll] = static_cast<short>(1000.f * sensorsCurr[3]);
  jointSensorData.temperatures[Joints::lShoulderRoll] = static_cast<unsigned char>(sensorsTemp[3]);
  jointSensorData.status[Joints::lShoulderRoll] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[3]));

  jointSensorData.angles[Joints::lShoulderPitch] = sensorsPos[2] - theJointCalibration.offsets[Joints::lShoulderPitch];
  jointSensorData.currents[Joints::lShoulderPitch] = static_cast<short>(1000.f * sensorsCurr[2]);
  jointSensorData.temperatures[Joints::lShoulderPitch] = static_cast<unsigned char>(sensorsTemp[2]);
  jointSensorData.status[Joints::lShoulderPitch] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[2]));

  jointSensorData.angles[Joints::headPitch] = sensorsPos[1] - theJointCalibration.offsets[Joints::headPitch];
  jointSensorData.currents[Joints::headPitch] = static_cast<short>(1000.f * sensorsCurr[1]);
  jointSensorData.temperatures[Joints::headPitch] = static_cast<unsigned char>(sensorsTemp[1]);
  jointSensorData.status[Joints::headPitch] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[1]));

  jointSensorData.angles[Joints::headYaw] = sensorsPos[0] - theJointCalibration.offsets[Joints::headYaw];
  jointSensorData.currents[Joints::headYaw] = static_cast<short>(1000.f * sensorsCurr[0]);
  jointSensorData.temperatures[Joints::headYaw] = static_cast<unsigned char>(sensorsTemp[0]);
  jointSensorData.status[Joints::headYaw] = static_cast<JointSensorData::TemperatureStatus>(*reinterpret_cast<int*>(&sensorsStat[0]));


  jointSensorData.timestamp = theFrameInfo.time;
}

void NaoProvider::update(KeyStates& keyStates)
{ 
  // Check: KeyStates.h
  //
  // and map with Touch (from lola)
  // 0. ChestBoard/Button
  // 1. Head/Touch/Front
  // 2. Head/Touch/Middle
  // 3. Head/Touch/Rear
  // 4. LFoot/Bumper/Left
  // 5. LFoot/Bumper/Right
  // 6. LHand/Touch/Back
  // 7. LHand/Touch/Left
  // 8. LHand/Touch/Right
  // 9. RFoot/Bumper/Left
  // 10. RFoot/Bumper/Right
  // 11. RHand/Touch/Back
  // 12. RHand/Touch/Left
  // 13. RHand/Touch/Right

  keyStates.pressed[KeyStates::chest] = theInstance->lrd.touch[0];
  keyStates.pressed[KeyStates::headFront] = theInstance->lrd.touch[1];
  keyStates.pressed[KeyStates::headMiddle] = theInstance->lrd.touch[2];
  keyStates.pressed[KeyStates::headRear] = theInstance->lrd.touch[3];
  keyStates.pressed[KeyStates::lFootLeft] = theInstance->lrd.touch[4];
  keyStates.pressed[KeyStates::lFootRight] = theInstance->lrd.touch[5];
  keyStates.pressed[KeyStates::lHandBack] = theInstance->lrd.touch[6];
  keyStates.pressed[KeyStates::lHandLeft] = theInstance->lrd.touch[7];
  keyStates.pressed[KeyStates::lHandRight] = theInstance->lrd.touch[8];
  keyStates.pressed[KeyStates::rFootLeft] = theInstance->lrd.touch[9];
  keyStates.pressed[KeyStates::rFootRight] = theInstance->lrd.touch[10];
  keyStates.pressed[KeyStates::rHandBack] = theInstance->lrd.touch[11];
  keyStates.pressed[KeyStates::rHandLeft] = theInstance->lrd.touch[12];
  keyStates.pressed[KeyStates::rHandRight] = theInstance->lrd.touch[13];

}

void NaoProvider::update(OpponentTeamInfo& opponentTeamInfo)
{
  (RoboCup::TeamInfo&) opponentTeamInfo = gameControlData.teams[gameControlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 1 : 0];
}

void NaoProvider::update(OwnTeamInfo& ownTeamInfo)
{
  (RoboCup::TeamInfo&) ownTeamInfo = gameControlData.teams[gameControlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
}

void NaoProvider::update(RawGameInfo& rawGameInfo)
{
  memcpy(&(RoboCup::RoboCupGameControlData&) rawGameInfo, &gameControlData, sizeof(gameControlData));
  rawGameInfo.timeLastPackageReceived = gameControlTimeStamp;
}

void NaoProvider::update(RobotInfo& robotInfo)
{
  RoboCup::TeamInfo& team = gameControlData.teams[gameControlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
  (RoboCup::RobotInfo&) robotInfo = team.players[Global::getSettings().playerNumber - 1];
  robotInfo.number = Global::getSettings().playerNumber;
  
  DEBUG_RESPONSE_ONCE("module:NaoProvider:robotInfo")
  {
    OUTPUT(idRobotInfo, bin, robotInfo);
  }
}

void NaoProvider::update(SystemSensorData& systemSensorData)
{
  
  if(theFrameInfo.getTimeSince(lastBodyTemperatureReadTime) * 1000 > 10)
  {
    lastBodyTemperatureReadTime = theFrameInfo.time;
    systemSensorData.cpuTemperature = naoBody.getCPUTemperature();
  }
  systemSensorData.batteryCurrent = theInstance->lrd.battery[2];
  systemSensorData.batteryLevel = theInstance->lrd.battery[0];
  systemSensorData.batteryTemperature = theInstance->lrd.battery[3];
  const short statusValue = static_cast<short>(theInstance->lrd.battery[1]);
  systemSensorData.batteryCharging = statusValue & 0b10000000;
}

#endif // TARGET_ROBOT
