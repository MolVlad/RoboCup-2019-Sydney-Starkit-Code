option(HeadControl)
{
  common_transition
  {
    if(!theGroundContactState.contact && theGameInfo.state != STATE_INITIAL)
      goto lookForward;

    switch(theHeadControlMode)
    {
      case HeadControl::off:
        goto off;
      case HeadControl::lookForward:
        goto lookForward;
      case HeadControl::lookAtBall:
        goto lookAtBall;
      case HeadControl::lookAtLandmark:
        goto lookAtLandmark;
      case HeadControl::lookLeftAndRight:
         goto lookLeftAndRight;
      default:
        goto none;
    }
  }

  initial_state(none) {}
  state(off) {action SetHeadPanTilt(JointAngles::off, JointAngles::off, 0.f);}
  state(lookForward) {action LookForward();}
  state(lookAtBall) {action lookAtBall();}
  state(lookAtLandmark) {action lookAtLandmark();}
  state(lookLeftAndRight) {action lookLeftAndRight();}

}

struct HeadControl
{
  ENUM(Mode,
  {,
    none,
    off,
    lookForward,
    lookAtBall,
    lookAtLandmark,
    lookLeftAndRight,
  });
};

HeadControl::Mode theHeadControlMode = HeadControl::Mode::none; /**< The head control mode executed by the option HeadControl. */
