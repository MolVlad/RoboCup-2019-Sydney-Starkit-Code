/**
 * @file Platform/Nao/NaoBody.cpp
 * Declaration of a class for accessing the body of the nao via NaoQi/libbhuman.
 * @author Colin Graf
 * @author jeff
 */

#include <sys/socket.h>

// TODO remove (these were for the shared memory)
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <unistd.h>
// end remove

#include <cerrno>
#include <cstdio>

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cstring>
#include <pthread.h>

#include "NaoBody.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"

#include "libbhuman/bhuman.h"
#include "../Util/Buildchain/gcc/include/msgpack.hpp"
#include "Tools/spqrSocketMethods/SPQRSocket.h"
#include "Modules/Infrastructure/NaoProvider/NaoProvider.h"


void replaceAll(std::string& str, const std::string& from, const std::string& to) {
  if(from.empty())
    return;
  size_t start_pos = 0;
  while((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
  }
}

void unpackLolaData(LolaRead *lrd , msgpack::object obj) {

  msgpack::object_kv* kv;
  int mapSize = obj.via.map.size;

  for(int i=0; i<mapSize; i++) {
    kv = obj.via.map.ptr + i;
    std::stringstream nss;
    nss<<kv->key;
    std::string a = nss.str();

    for(uint32_t k=0; k < kv->val.via.array.size; k++) {
      std::stringstream nss2;
      nss2<<*(kv->val.via.array.ptr +k);
      std::string tp = nss2.str();
      replaceAll(tp,"\"","");

      if(a == "\"RobotConfig\""){
        if(k == 0) {
          strcpy(lrd->bodyId,tp.c_str());
        } else if(k == 1) {
          lrd->bodyVersion = NAOVersion::V6;
        } else if(k == 2) {
          strcpy(lrd->headId,tp.c_str());
        } else {
          lrd->headVersion = NAOVersion::V6;
        }

      } else if(a == "\"Accelerometer\"") {
        lrd->accelerometer[k] = std::stof(tp);

      } else if(a == "\"Angles\"") {
        lrd->angles[k] = std::stof(tp);

      } else if(a == "\"Battery\"") {
        lrd->battery[k] = std::stof(tp);

      } else if(a == "\"Current\"") {
        lrd->current[k] = std::stof(tp);

      } else if(a == "\"FSR\"") {
        lrd->FSR[k] = std::stof(tp);

      } else if(a == "\"Gyroscope\"") {
        lrd->gyroscope[k] = std::stof(tp);

      } else if(a == "\"Position\"") {
        lrd->positions[k] = std::stof(tp);

      } else if(a == "\"Sonar\"") {
        lrd->sonar[k] = std::stof(tp);

      } else if(a == "\"Stiffness\"") {
        lrd->stiffness[k] = std::stof(tp);

      } else if(a == "\"Temperature\"") {
        lrd->temperature[k] = std::stof(tp);

      } else if(a == "\"Touch\"") {
        lrd->touch[k] = std::stof(tp);

      } else if(a == "\"Status\"") {
        //TODO SPQRfill this
      }
    }
  }
}

static char socketname[14] = "/tmp/robocup\0";

class NaoBodyAccess
{
public:
  //TODO remove
  int fd = -1; /**< The file descriptor for the shared memory block. */
  sem_t* sem = SEM_FAILED; /**< The semaphore used for synchronizing to the NaoQi DCM. */
  LBHData* lbhData = (LBHData*)MAP_FAILED; /**< The pointer to the mapped shared memory block. */

  // LOLA
  LolaRead lrd;
  int mySocket;

  bool isStopping = false;

  
  ~NaoBodyAccess()
  {
    cleanup();
  }

  void stopBhumand(bool stop)
  {
    if (stop)
      isStopping = true;
    else
      isStopping = false;
    return;
  }

  bool returnIsStopping()
  {
    return isStopping;
  }

  bool init_lola() {

    const int size = 896; /* LOLA buffer size (fixed size) */
    // start lola socket (see SPQRSocket.h)
    int sockLola = make_named_socket(socketname);

    char *buff = new char[size];
    int byteRead = 0;
    int ret = 0;

    // receive
    while(byteRead < size) {
      ret = recv(sockLola, buff + byteRead, size - byteRead,0);
      byteRead += ret;
    }

    msgpack::unpacked result;
    unpack(result, buff,size);
    msgpack::object obj(result.get()); // Get msgpack::object from msgpack::unpacked (shallow copy)


    unpackLolaData(&lrd,obj);

    // NOTE removed the for cycle over this two, all zeros, there is no need for it
    std::vector<float> position;
    std::vector<float> stiffness;
    
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    position.push_back(0);
    stiffness.push_back(0);
    
    // NOTE removed the for cycle over this two, all zeros except i==2, there is no need for it
    std::vector<float>lear(10, 0.);
    std::vector<float>rear(10, 0.);
    lear.at(2) = 1.;
    rear.at(2) = 1.;

    std::vector<float> chest;

    chest.push_back(0);
    chest.push_back(0);
    chest.push_back(1);

    std::map<std::string, std::vector<float>> v { {"Chest", chest} , {"Position", position}, {"Stiffness", stiffness} };
    std::stringstream buffer;
    msgpack::pack(buffer, v);

    // deserialize it.
    /*msgpack::object_handle oh =
      msgpack::unpack(sbuf.data(), sbuf.size());

      // print the deserialized object.
      msgpack::object obj2 = oh.get();
      std::cout << obj2 << std::endl;  //=> ["Hello", "MessagePack"]
    */

    byteRead = 0;
    ret = 0;
    usleep(12000); // TODO SPQR remove

    while(byteRead < buffer.str().size()){
      const char *c = buffer.str().c_str();
      ret = send(sockLola, c, buffer.str().size(),0);
      printf ("Error %d opening socket.\n", errno);
      std::cout<<ret<<std::endl;
      byteRead += ret;

    }

    std::cout<<shutdown(sockLola,2)<<"\n";
    return true;

  }


  void cleanup()
  {
    if(lbhData != MAP_FAILED)
      {
        munmap(lbhData, sizeof(LBHData));
        lbhData = (LBHData*)MAP_FAILED;
      }
    if(fd != -1)
      {
        close(fd);
        fd = -1;
      }
    if(sem != SEM_FAILED)
      {
        sem_close(sem);
        sem = SEM_FAILED;
      }
  }
} naoBodyAccess;

NaoBody::~NaoBody()
{
  if(fdCpuTemp)
    fclose(fdCpuTemp);
}

bool NaoBody::init_lola()
{
  return naoBodyAccess.init_lola();
}


bool NaoBody::returnIsStopping()
{
  return naoBodyAccess.returnIsStopping();
}


void NaoBody::stopBhumand(bool stop)
{
  return naoBodyAccess.stopBhumand(stop);
}

void NaoBody::cleanup()
{
  naoBodyAccess.cleanup();
}

void NaoBody::setCrashed(int termSignal)
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  naoBodyAccess.lbhData->state = BHState(termSignal);
}

bool NaoBody::wait()
{

  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  ASSERT(naoBodyAccess.sem != SEM_FAILED);
  do
    {
      if(sem_wait(naoBodyAccess.sem) == -1)
        {
          bool success = false;
          while(errno == 516)
            {
              if(sem_wait(naoBodyAccess.sem) == -1)
                {
                  ASSERT(false);
                  continue;
                }
              else
                {
                  success = true;
                  break;
                }
            }
          if(!success)
            {
              ASSERT(false);
              return false;
            }
        }
    }
  while(naoBodyAccess.lbhData->readingSensors == naoBodyAccess.lbhData->newestSensors);
  naoBodyAccess.lbhData->readingSensors = naoBodyAccess.lbhData->newestSensors;

  static bool shout = true;
  if(shout)
    {
      shout = false;
      printf("NaoQi is working\n");
    }

  return true;
}

const char* NaoBody::getHeadId() const
{
  return naoBodyAccess.lrd.headId;
}

const char* NaoBody::getBodyId() const
{
  return naoBodyAccess.lrd.bodyId;
}

RobotInfo::NaoVersion NaoBody::getHeadVersion()
{
  return static_cast<RobotInfo::NaoVersion>(NAOVersion::V6);
}

RobotInfo::NaoVersion NaoBody::getBodyVersion()
{
  return static_cast<RobotInfo::NaoVersion>(NAOVersion::V6);
}

RobotInfo::NaoType NaoBody::getHeadType()
{
  return static_cast<RobotInfo::NaoType>(NAOType::H25);
}

RobotInfo::NaoType NaoBody::getBodyType()
{
  return static_cast<RobotInfo::NaoType>(NAOType::H25);
}

float* NaoBody::getSensors()
{
  return naoBodyAccess.lrd.positions;
}

const RoboCup::RoboCupGameControlData& NaoBody::getGameControlData() const
{
  RoboCup::RoboCupGameControlData rbc;
  return rbc;
}

float NaoBody::getCPUTemperature()
{

  // ma porco...
  return 32.0;
}

bool NaoBody::getWlanStatus()
{
  return access("/sys/class/net/wlan0", F_OK) == 0;
}

void NaoBody::openActuators(float*& actuators)
{
  return;
}


void NaoBody::closeActuators()
{
  return;
}

void NaoBody::setTeamInfo(int teamNumber, int teamColor, int playerNumber)
{
  return;
}
