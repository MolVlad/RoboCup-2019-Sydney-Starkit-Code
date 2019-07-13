
option(Looking, (float) alpha, (float) pos_x, (float) pos_y)
{
    initial_state(start)
    {
      transition
      {
        if ((std::abs(theRobotPose.translation.angle() - alpha) <= 15_deg) && (std::abs(theRobotPose.translation.x() - pos_x) <= 500.f) && (std::abs(theRobotPose.translation.y() - pos_y) <= 500.f))
          goto observe;
      }
      action
      {
        WalkToTargetPathPlanner(Pose2f(50.f, 50.f, 50.f), Pose2f(alpha, pos_x, pos_y));
      }
    }

    state(observe)
    {
      transition
      {
        if ((std::abs(theRobotPose.translation.angle() - alpha) > 15_deg) || (std::abs(theRobotPose.translation.x() - pos_x) > 500.f) || (std::abs(theRobotPose.translation.y() - pos_y) > 500.f))
          goto start;
      }
      action
      {
        theHeadControlMode = HeadControl::lookLeftAndRight;
        Stand();
      }
    }
}