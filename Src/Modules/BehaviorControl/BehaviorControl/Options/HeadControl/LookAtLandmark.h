
option(lookAtLandmark)
{
    initial_state(lookAtLandmark)
    {
        action
        {
         /*MIO   Pose2f center(theLibCodeRelease.glob2Rel(0, 0,0));
            Pose2f goal(theLibCodeRelease.glob2Rel(-SPQR::FIELD_DIMENSION_X, 0,0) );

            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::targetMode;
            if(theRobotPose.translation.x() < -0.3*SPQR::FIELD_DIMENSION_X)
            {
                theHeadMotionRequest.target.x() = goal.translation.x();
                theHeadMotionRequest.target.y() = goal.translation.y();
            }
            else
            {
                theHeadMotionRequest.target.x() = center.translation.x();
                theHeadMotionRequest.target.y() = center.translation.y();
            }
            theHeadMotionRequest.target.z() = 35;
            theHeadMotionRequest.speed = 1;*/
        }
    }

}
