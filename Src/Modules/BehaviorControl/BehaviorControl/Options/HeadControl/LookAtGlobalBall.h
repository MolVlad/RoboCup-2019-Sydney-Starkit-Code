option(lookAtGlobalBall)
{
    initial_state(lookAtGlobalBall)
    {

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::targetMode;
            //Pose2f globalBall = theLibCodeRelease.glob2Rel(theTeamBallModel.position.x(),theTeamBallModel.position.y());
            Pose2f globalBall;
            if(theGameInfo.state == STATE_SET)
                globalBall = theLibCodeRelease.glob2Rel(0.f, 0.f);
            else
                globalBall = theLibCodeRelease.glob2Rel(theTeamBallModel.position.x(),theTeamBallModel.position.y());
            
            theHeadMotionRequest.target.x() = globalBall.translation.x();
            theHeadMotionRequest.target.y() = globalBall.translation.y();

            
            theHeadMotionRequest.target.z() = 35;
            theHeadMotionRequest.speed = 1;
        }
    }

}
