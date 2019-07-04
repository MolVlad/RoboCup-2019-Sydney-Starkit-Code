#ifndef OURDEFINITIONS_H
#define OURDEFINITIONS_H

//   NETWORK ----------------------------------------------------------------------------

//#define TWO_TEAMS_NETWORK //if we want to play a real game in lab with two teams. This divides the team's communications.
//#define UNSTABLE_NETWORK //if we want to simulate a bad network in the simulated environment.
#define IW_INTERFACE "wlan0"
//#define SLOW_SEND //in case of bad network we slow the packets sent. Go to TeamDataSender.cpp for more details.
#define GOOD_CONVEX_QUALITY 60 //above this quality we don't slow anything
#define NUMBER_OF_FRAMES_TO_WAIT 10 //number of frames to wait in case of bad quality network ( One frame is equal to 20ms, normally we send packets at 5Hz )

#define IP_FOR_UNSTABLE "192.168.1.185"
#endif
