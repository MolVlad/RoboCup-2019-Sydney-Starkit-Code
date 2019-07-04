/**
 * @file Representations/BehaviorControl/Role.h
 *
 * Declaration of the representation of a robot's behavior role
 *
 * @author Tim Laue, Andreas Stolpmann
 */

#pragma once

#include "RoleCheck.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Pose2f.h"
/**
 * @struct Role
 * Representation of a robot's behavior role
 */
STREAMABLE(Role,
{
  /** The different roles */
  ENUM(RoleType,
  {,
   undefined,
   goalie,
   striker,
   defender,
   supporter,
   jolly,
   searcher_1,
   searcher_2,
   searcher_3,
   searcher_4,
   penaltyStriker,
   penaltyKeeper,
   planStriker,
   planJolly,
   none,

  });

  bool isGoalkeeper() const;
  
  ENUM(Context,
   {,
       no_context, //TODO CAPIRE COME SOSTITUIRE L' =1 che c'era
       playing,
       search_for_ball,
   });
   
  

  CHECK_OSCILLATION(role, RoleType, undefined, "Role", 5000)
  /** Draws the current role next to the robot on the field view (in local robot coordinates) */
  void draw() const,

  /** Instance of role */
  (RoleType)(undefined) role,
  (RoleType)(undefined) lastRole,
  (Context)(no_context) current_context,
  (std::vector<int>)(5,0) utility_vector,
  (std::vector<Pose2f>)(std::vector<Pose2f>(5,Pose2f())) robots_poses,
});
