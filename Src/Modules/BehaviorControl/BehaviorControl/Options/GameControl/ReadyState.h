#define R1X -4500
#define R1Y 0

#define R3X -2000
#define R3Y -1200

#define R2X -1200
#define R2Y 0

#define R4X -2000
#define R4Y 1200

#define R5X -3500
#define R5Y -1200

/** behavior for the ready state */
option(ReadyState)
{
  /* position has been reached -> stand and wait */
  initial_state(InitReady)
  {
    action
    {
      HeadControlMode(HeadControl::lookForward);
      Stand();
    }
    transition
    {
      goto WalkToTarget_Ready;
    }
  }
  state(WalkToTarget_Ready)
  {
    action
    {
      // LeftAndRigth doesn't work here
      HeadControlMode(HeadControl::lookLeftAndRight);

      if(theRobotInfo.number == 1)
        WalkToTargetPathPlanner(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f, R1X, R1Y));
      else if(theRobotInfo.number == 2)
        WalkToTargetPathPlanner(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f, R2X, R2Y));
      else if(theRobotInfo.number == 3)
        WalkToTargetPathPlanner(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f, R3X, R3Y));
      else if(theRobotInfo.number == 4)
        WalkToTargetPathPlanner(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f, R4X, R4Y));
      else if(theRobotInfo.number == 5)
        WalkToTargetPathPlanner(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f, R5X, R5Y));
    }
    transition
    {

    }
  }
  state(Completed_Ready)
  {
    action
    {
      // LeftAndRigth doesn't work here
      HeadControlMode(HeadControl::lookLeftAndRight);
      Stand();
    }
    transition
    {

    }
  }
  state(TurnToTarget_Ready)
  {
    action
    {
      HeadControlMode(HeadControl::lookLeftAndRight);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f, 0.f, 0.f));
    }
    transition
    {
      //if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
        //goto Completed_Ready;
    }
  }
}
