
option(lookAtBall)
{
    initial_state(lookAtBall)
    {
        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::targetMode;
#ifdef TARGET_ROBOT
            //TODO FIX VINCENZO - EMERGENCY MAKER FAIRE FIX
            {
                theHeadMotionRequest.target.x() = theBallModel.estimate.position.x();
                theHeadMotionRequest.target.y() = theBallModel.estimate.position.y();
            }
#else // in simulata non abbiamo accesso a ballpercept
            theHeadMotionRequest.target.x() = theBallModel.estimate.position.x();
            theHeadMotionRequest.target.y() = theBallModel.estimate.position.y();
#endif
            theHeadMotionRequest.target.z() = 35;
            theHeadMotionRequest.speed = 1;
        }
    }

}

