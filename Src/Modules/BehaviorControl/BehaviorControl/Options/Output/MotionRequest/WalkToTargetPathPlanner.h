/** Sets all members of the MotionRequest representation for executing a TargetMode-WalkRequest
 *  (i.e. Walk to a \c target at a \c speed)
 *  @param speed Walking speeds, in percentage.
 *  @param target Walking target, in mm and radians, Absolute.
 */
option(WalkToTargetPathPlanner, (const Pose2f&) speed, (const Pose2f&) target)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::walk)
        goto requestIsExecuted;
    }
    action
    {
      MotionRequest mr = thePathPlanner.plan(target,speed,false);
      assert(speed.translation.x()!=0.f && speed.translation.y()!=0.f && speed.rotation!=0.f);
      theMotionRequest.motion = mr.motion;
      theMotionRequest.walkRequest.mode = mr.walkRequest.mode;
      theMotionRequest.walkRequest.target = mr.walkRequest.target;
      theMotionRequest.walkRequest.speed = mr.walkRequest.speed;
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::walk)
        goto setRequest;
    }
    action
    {
      MotionRequest mr = thePathPlanner.plan(target,speed,false);
      theMotionRequest.motion = mr.motion;
      theMotionRequest.walkRequest.mode = mr.walkRequest.mode;
      theMotionRequest.walkRequest.target = mr.walkRequest.target;
      theMotionRequest.walkRequest.speed = mr.walkRequest.speed;
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    }
  }
}
