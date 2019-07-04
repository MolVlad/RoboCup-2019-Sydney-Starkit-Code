/**
 * @file LibCodeReleaseProvider.h
 */

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Tools/Math/Eigen.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/OurDefinitions.h"


MODULE(LibCodeReleaseProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamData),
  PROVIDES(LibCodeRelease),
});


class LibCodeReleaseProvider : public LibCodeReleaseProviderBase
{
  public:LibCodeReleaseProvider();

  void update(LibCodeRelease& libCodeRelease);
  bool between(float value, float min, float max);
  bool isValueBalanced(float currentValue, float target, float bound);
  float angleToTarget(float x, float y);
  float norm(float x, float y);
  Pose2f glob2Rel(float x, float y);  // TODO MARCO FIX
  Pose2f rel2Glob(float x, float y);
  float distance(Pose2f p1, Pose2f p2);
  float defenderDynamicY();
  float defenderDynamicDistance();
  bool defenderNearestBall();
  Pose2f angleAreaToPass(std::vector<Obstacle> opponents);

  bool isGoalieInStartingPosition();
  bool isBallInKickAwayRange();
  bool isGoalieInKickAwayRange();
  bool isBallInArea();
  bool isGoalieInArea();
  float getGoalieCoverAngleDisplacement();
  Pose2f getGoalieCoverPosition();
  bool isGoalieInAngle();
  float goalie_displacement;
  float angleToGoal;
  float angleToMyGoal;
  float penaltyAngle;
  float kickAngle;
  float correctionKickAngle;
  bool ballOutOnLeft;
  bool diveBool;
  Pose2f disambiguateCell(int cell);
  int discretizePose(float x, float y);
  float radiansToDegree(float x);


  Pose2f getReadyPose(bool kickoff, Role::RoleType rRole);

  ///////////ROBA NUOVA
  bool otherStateHasBall(int myPose, Vector2f ballPose);
  bool stateHasBall(Pose2f myPose, Vector2f ballPose);
  std::vector<Obstacle> globalizeObstacles(Pose2f myPose, std::vector<Obstacle> oldOpponents);
  std::vector<float> computeFreeAreas(Pose2f myPose, std::vector<Obstacle> opponents);
  std::vector<float> computeTarget(std::vector<float> freeAreas);
  Pose2f computeBetterTarget(float target, Pose2f robotPose);
  Pose2f approachPoint(Pose2f target, Pose2f globBall, bool behind);


  bool freeBallSight(Pose2f myPose, Vector2f ballPose, std::vector<Obstacle> opponents);
  bool canPass(Pose2f targetPose, Pose2f shootingPose, std::vector<Obstacle> opponents);



  Vector2f updateDefender();
  Vector2f updateSupporter();
  Vector2f updateGoalie();

  Vector3f getTriang3Points(Pose2f pa, Pose2f pb, Pose2f pc);


};
