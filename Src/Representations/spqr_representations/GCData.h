#pragma once

#include <vector>
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/Communication/BHumanMessage.h"
#include "Tools/Streams/Enum.h"



STREAMABLE(RobotInfoGC,
{
    public:
        float x,
        (int) penalty,              // penalty state of the player
        (int) secsTillUnpenalised,  // estimate of time till unpenalised
});

STREAMABLE(TeamInfoGC,
{
    public:
        float x,
        (int) teamNumber,           // unique team number
        (int) teamColour,           // colour of the team
        (int) score,                // team's score
        (int) penaltyShot,          // penalty shot counter
        (int) singleShots,         // bits represent penalty shot success
        (std::vector<RobotInfoGC>) players, // the team's players
        TeamInfo(){
            players = std::vector(6);
        }
});

STREAMABLE(GCData,
{
	public:

		float x,        	/**< The X position of the filtered robot pose */
        
        (int) version,              // version of the data structure
        (int) packetNumber,         // number incremented with each packet sent (with wraparound)
        (int) playersPerTeam,       // the number of players on a team
        (int) competitionPhase,     // phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
        (int) competitionType,      // type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_MIXEDTEAM)
        (int) gamePhase,            // phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
        (int) state,                // state of the game (STATE_READY, STATE_PLAYING, etc)
        (int) setPlay,              // active set play (SET_PLAY_NONE, SET_PLAY_GOAL_FREE_KICK, etc)
        (int) firstHalf,            // 1 = game in first half, 0 otherwise
        (int) kickingTeam,          // the team number of the next team to kick off, free kick etc
        (int) secsRemaining,        // estimate of number of seconds remaining in the half
        (int) secondaryTime,        // number of seconds shown as secondary time (remaining ready, until free ball, etc)
  		
        // BUMAN VARIABLEs
        //(int) chestButtonPressCounter,
        (int) previousState, /**< The game state during the previous cycle. Used to detect when LEDs have to be updated. */
        (int) previousGamePhase, /**< The game phase during the previous cycle. Used to detect when LEDs have to be updated. */
        (int) previousKickingTeam, /**< The kicking team during the previous cycle. Used to detect when LEDs have to be updated. */
        (int) previousTeamColour, /**< The team colour during the previous cycle. Used to detect when LEDs have to be updated. */
        (int) previousPenalty, /**< The penalty set during the previous cycle. Used to detect when LEDs have to be updated. */

        (std::vector<TeamInfoGC>) teams,
        
        GCData(){
            teams = std::vector(2);
            previousState =-1;
            previousGamePhase = -1;
            previousKickingTeam = -1;
            previousTeamColour = -1;
            previousPenalty = -1;
            initialTime = 0;
        };
});

