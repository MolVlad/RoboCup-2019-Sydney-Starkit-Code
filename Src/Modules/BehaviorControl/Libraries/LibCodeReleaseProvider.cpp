/**
 * @file LibCodeRelease.cpp
 */


#include "LibCodeReleaseProvider.h"
#include <iostream>

MAKE_MODULE(LibCodeReleaseProvider, behaviorControl);

LibCodeReleaseProvider::LibCodeReleaseProvider(): goalie_displacement(300.f),
        angleToGoal(0.f), angleToMyGoal(0.f), kickAngle(0.f), correctionKickAngle(0.f), ballOutOnLeft(false), diveBool(false)
{
    SPQR::ConfigurationParameters();
}

float globBallY;

void LibCodeReleaseProvider::update(LibCodeRelease& libCodeRelease)
{
  libCodeRelease.timeSinceBallWasSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  libCodeRelease.angleToGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  libCodeRelease.isGoalieInStartingPosition = isGoalieInStartingPosition();
  libCodeRelease.isBallInArea = isBallInArea();
  libCodeRelease.getGoalieCoverPosition = getGoalieCoverPosition();
  libCodeRelease.isGoalieInAngle = isGoalieInAngle();
  libCodeRelease.isGoalieInArea = isGoalieInArea();
  libCodeRelease.getGoalieCoverAngleDisplacement = getGoalieCoverAngleDisplacement();
  libCodeRelease.isGoalieInKickAwayRange = isGoalieInKickAwayRange();
  libCodeRelease.isBallInKickAwayRange = isBallInKickAwayRange();
  libCodeRelease.between = [&](float value, float min, float max) -> bool
  {
    return value >= min && value <= max;
  };

  libCodeRelease.norm = [&](float x, float y) -> float
  {
    return sqrt((x*x) + (y*y));
  };
    libCodeRelease.distance = [&] (Pose2f p1, Pose2f p2)-> float
    {
    float diffX = p1.translation.x() - p2.translation.x();
    float diffY = p1.translation.y() - p2.translation.y();

    return sqrt((diffX*diffX)+(diffY*diffY));
    };
  libCodeRelease.isValueBalanced = [&](float currentValue, float target, float bound) -> bool
  {
      float minErr = currentValue - (target - bound);
      float maxErr = currentValue - (target + bound);

      if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
          return true;
      else
          return false;
  };

  libCodeRelease.angleToTarget = [&](float x, float y) -> float
  {
    Pose2f relativePosition = glob2Rel(x,y);
    //std::cerr << "y relativa: "<< relativePosition.translation.y() << ", x relativa: "<<relativePosition.translation.x() << std::endl;
    //return radiansToDegree(atan2f(relativePosition.translation.y(), relativePosition.translation.x()));
    return (atan2f(relativePosition.translation.y(), relativePosition.translation.x()));

  };

  libCodeRelease.disambiguateCell = [&](int cell) -> Pose2f
  {
    Pose2f myPose;
    int myY = (int)(cell/18);
    int myX = (int)(cell%18);
    myPose.translation.x() = ((500.f * myX) + 250.) -4500.;
    myPose.translation.y() = ((500.f * myY) + 250.) -3000.;

    return myPose;
  };

  libCodeRelease.discretizePose = [&](float x, float y) -> int
  {

    int myX = (int)((x + 4500.)/500.);
    int myY = (int)((y + 3000.)/500.);
    int discretizedPose;
    if(myX %18 == 0 && myX != 0){
      discretizedPose = myX + 18*myY -1;
    }else{
      discretizedPose = myX + 18*myY;
    }

    return discretizedPose;
  };

  libCodeRelease.glob2Rel = [&](float x, float y) -> Pose2f
  {
      Vector2f result;
      float theta = 0;
      float tempX = x - theRobotPose.translation.x();
      float tempY = y - theRobotPose.translation.y();

      result.x() = tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation);
      result.y() = -tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation);

      return Pose2f(theta , result.x(),result.y());
  };

  libCodeRelease.rel2Glob = [&](float x, float y) -> Pose2f
  {
      Vector2f result;
      float rho = sqrt((x * x) + (y * y));

      result.x() = theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x)));
      result.y() = theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x)));

      return Pose2f(result.x(),result.y());
  };





///////////////////////////////////////////////////////////////////////////ROBA NUOVA/////////////////////////////////
    libCodeRelease.defenderDynamicY = [&]() -> float
    {   
        float x2 = theTeamBallModel.position.x();        
        float y2 = theTeamBallModel.position.y();
        float x1 = -4500.f;   // first goalpost for defender
        float y1 = (y2/(std::abs(y2)+1))*750.f;   // first goalpost for defender        
        float defenderBallY = (( libCodeRelease.defenderPosition.x()-x1 )*( y2-y1 ))/( x2-x1 ) + y1;

        return defenderBallY -(y2/(std::abs(y2)+1))*100.f;
    };

    libCodeRelease.defenderDynamicDistance = [&]() -> float
    {   
        float x1 = theRobotPose.translation.x();
        float y1 = theRobotPose.translation.y();
        float x3 = theTeamBallModel.position.x();
        float y3 = theTeamBallModel.position.y();
        float x2 = -4500.f;   // first goalpost for defender
        float y2 = (y3/(std::abs(y3)+1))*750.f;  // first goalpost for defender
        float m = (y1-y2)/(x1-x2) ;
        float q = y1 - (((y1-y2)/(x1-x2))*x1) ;

        
        float distance = std::abs( y3 - (m*x3 +q) )/(std::sqrt( 1 + (m*m) ));

        return distance;
    };

    libCodeRelease.defenderNearestBall = [&]() -> bool
    {   
        Vector2f ballPosition = theBallModel.estimate.position;
        float myDistanceToBall = theBallModel.estimate.position.norm();
        bool iAmMostNearPlayer = true;
        for(int i = 0; i < theTeamData.teammates.size(); i++){
            if((theTeamData.teammates.at(i).theRobotPose.translation -
                    theTeamBallModel.position).norm() < (myDistanceToBall+400.f) && theTeamData.teammates.at(i).number != 1){
                iAmMostNearPlayer = false;
            }
        }
        return iAmMostNearPlayer;
    };

    libCodeRelease.angleAreaToPass = [&](std::vector<Obstacle> myOpponents) -> Pose2f
    {   
        Pose2f swapper;
        Pose2f bestTarget = Pose2f(4500.f, 0.f);  // absolute position standard to make goal
        float lastM ;
        float maxRange = 0 ;
        std::vector<Pose2f> opponents;
        //globalizing obstacles and delete obstacles behind me
        for(int i = 0; i < myOpponents.size(); i++){
            Pose2f obstacleGlob =  rel2Glob(myOpponents.at(i).center.x(),myOpponents.at(i).center.y());
            if(obstacleGlob.translation.x() > theRobotPose.translation.x() ){
                opponents.push_back(obstacleGlob);
            }
        }
        //ordering obstacles by y 
        for(int i = 0; i < opponents.size(); i++){
            for(int k = 0; k < opponents.size(); k++){
                if(opponents.at(i).translation.y() > opponents.at(k).translation.y() ){
                    swapper = opponents.at(k);
                    opponents.at(k) = opponents.at(i);
                    opponents.at(i) = swapper;
                }
            }          
        }
        // find best position to kick based on angular coefficients
        for(int i = 0; i < opponents.size(); i++){
            if( i == 0 ){
                float x1 = theRobotPose.translation.x();
                float y1 = theRobotPose.translation.y();
                float x2 = opponents.at(i).translation.x();
                float y2 = opponents.at(i).translation.y();
                lastM = (y2-y1)/(x2-x1) ;
                swapper = opponents.at(i);
            }
            else{
                float x1 = theRobotPose.translation.x();
                float y1 = theRobotPose.translation.y();
                float x2 = opponents.at(i).translation.x();
                float y2 = opponents.at(i).translation.y();
                float actualM = (y2-y1)/(x2-x1) ;         
                // std::cout<<"maxRange n= "<<i<<" val= "<< std::abs(lastM - actualM) <<std::endl;      
                if( maxRange < std::abs(lastM - actualM)){
                    float x3 = swapper.translation.x();
                    float y3 = swapper.translation.y();
                    
                    Pose2f possibleTarget = Pose2f( (x2+x3)/2 ,(y2+y3)/2 );
                    float m = (y1-possibleTarget.translation.y() )/(x1-possibleTarget.translation.x() ) ;
                    float q = y1 - (((y1-possibleTarget.translation.y() )/(x1-possibleTarget.translation.x()))*x1) ;
                    bool free = true;
                    // check if i have obstacles in my possibleTarget direction
                    for(const auto& obstacle : opponents){
                        float x4 = obstacle.translation.x();
                        float y4 = obstacle.translation.y();
                        float distanceFree = std::abs( y4 - (m*x4 +q) )/(std::sqrt( 1 + (m*m) ));
                        if( distanceFree <  300.f){
                            free = false;
                        }                       
                    }
                    if(free) bestTarget = Pose2f( (x2+x3)/2 ,(y2+y3)/2 );
                    maxRange = std::abs(lastM - actualM);
                }
                lastM = actualM;
                swapper = opponents.at(i);
            }
            // if my bestTarget is the goalPos check the obstacles and kick frontal
            if(bestTarget == Pose2f(4500.f, 0.f) && theRobotPose.translation.x() < 2300.f){
                float x1 = theRobotPose.translation.x();
                float y1 = theRobotPose.translation.y();
                float x2 = 4500.f;
                float y2 = 0.f;
                float m = (y1-y2 )/(x1-x2 ) ;
                float q = y1 - (((y1-y2 )/(x1-x2))*x1) ;
                bool free = true;
                // check if i have obstacles in my possibleTarget direction
                for(const auto& obstacle : opponents){
                    float x4 = obstacle.translation.x();
                    float y4 = obstacle.translation.y();
                    float distanceFree = std::abs( y4 - (m*x4 +q) )/(std::sqrt( 1 + (m*m) ));
                    if( distanceFree <  300.f){
                        free = false;
                    }                       
                }
                if(!free) bestTarget = Pose2f( theRobotPose.translation.x() + 1000.f , theRobotPose.translation.y() );
            }  
        }
        return bestTarget;
    };



  libCodeRelease.otherStateHasBall = [&](int myPose, Vector2f ballPose) -> bool{

    int ballCell= discretizePose(ballPose.x(),ballPose.y());

    if(myPose == ballCell){
        return true;
    }else{
        return false;
    }
  };

  libCodeRelease.canPass = [&](Pose2f targetPose, Pose2f shootingPose, std::vector<Obstacle> opponents) -> bool
  {
    float m = 0;
    bool sameX = false;
    bool sameY = false;
    //se hanno la stessa x (più o meno)
    if(std::abs(shootingPose.translation.x() - targetPose.translation.x()) <= 0.5){
      sameX = true;
    }
    else{
      float m = shootingPose.translation.y() - targetPose.translation.y();
      if(std::abs(m) <= 0.5 ){
        sameY = true;
      }
      else{
        m /= shootingPose.translation.x() - targetPose.translation.x();
      }
    }
    float q = shootingPose.translation.y() - m * shootingPose.translation.x();
    float qThreshold = 100.f;
    //TODO inserire il vettore degli opponents
    for(auto const& opponent : opponents){
      if(sameX){
        if(std::abs(opponent.center.x() - shootingPose.translation.x()) < qThreshold){
          std::cout<<"1"<<std::endl;
          return false;
        }
      }
      else if(sameY){
        if(std::abs(opponent.center.y() - shootingPose.translation.y()) < qThreshold){
          std::cout<<"2"<<std::endl;
          return false;
        }
      }
      else{
        for(int i = -50; i < 50; i++){
          //se l'opponent si trova su una retta del fascio
          if((opponent.center.y() - m * opponent.center.x() - q - (float) i * 10.f) <= 0.5f ){
            std::cout<<"3"<<std::endl;
            return false;
          }
        }
      }
    }
    return true;

  };


  libCodeRelease.stateHasBall = [&](Pose2f myPose, Vector2f ballPose) -> bool {

      int myCell = discretizePose(myPose.translation.x(),myPose.translation.y());
      int ballCell= discretizePose(ballPose.x(),ballPose.y());

      if(myCell == ballCell || theBallModel.estimate.position.norm() < 500.f ){
          return true;
      }else{
          return false;
      }
  };

  libCodeRelease.globalizeObstacles = [&](Pose2f myPose, std::vector<Obstacle> oldOpponents) -> std::vector<Obstacle> {

      int i;
      std::vector<Obstacle> opponents;
      for(i = 0; i < oldOpponents.size(); i++){
          Pose2f newLeft = rel2Glob(oldOpponents.at(i).left.x(),oldOpponents.at(i).left.y());
          Pose2f newCenter = rel2Glob(oldOpponents.at(i).center.x(),oldOpponents.at(i).center.y());
          Pose2f newRight =  rel2Glob(oldOpponents.at(i).right.x(),oldOpponents.at(i).right.y());
          oldOpponents.at(i).left.x() = newLeft.translation.x();
          oldOpponents.at(i).left.y() = newLeft.translation.y();

          oldOpponents.at(i).center.x() = newCenter.translation.x();
          oldOpponents.at(i).center.y() = newCenter.translation.y();

          oldOpponents.at(i).right.x() = newRight.translation.x();
          oldOpponents.at(i).right.y() = newRight.translation.y();
          if(oldOpponents.at(i).center.x() >  myPose.translation.x()){

              opponents.push_back(oldOpponents.at(i));
          }

      }

      return opponents;
  };

  libCodeRelease.freeBallSight = [&](Pose2f myPose,Vector2f ballPose, std::vector<Obstacle> opponents) -> bool{
    float ballDist = norm(myPose.translation.x() - ballPose.x(), myPose.translation.y() - ballPose.y());
    int i;
    float oppDist;
    bool sameSide;


    for(i = 0; i < opponents.size(); i++){

      if((myPose.translation.x() - ballPose.x()) >= 0 && (myPose.translation.x() - opponents.at(i).center.x()) >= 0){
        sameSide = true;
      } else if((myPose.translation.x() - ballPose.x()) <= 0 && (myPose.translation.x() - opponents.at(i).center.x()) <= 0){
        sameSide = true;
      } else {
        sameSide = false;
      }
      oppDist = norm(myPose.translation.x() - opponents.at(i).center.x(),
        myPose.translation.y() - opponents.at(i).center.y());

      if(oppDist < ballDist && sameSide == true){

        float divX = myPose.translation.x() - ballPose.x();
        if(divX == 0){
          divX = 0.1f;
        }

        float divY = myPose.translation.y() - ballPose.y();
        if(divY == 0){
          divY = 0.1f;
        }

        float numX =  myPose.translation.x() - opponents.at(i).center.x();
        float numY = myPose.translation.y() - opponents.at(i).center.y();

        float rightEqside = (numY/divY) * divX;

        //std::cout<<"numX "<<numX<<" rightEqside"<<rightEqside<<std::endl;
        if(numX > rightEqside -350 && numX < rightEqside + 350){
          return false;
        }

      }


    }
    return true;

  };

   libCodeRelease.computeFreeAreas = [&](Pose2f myPose, std::vector<Obstacle> opponents) -> std::vector<float>{
    Pose2f goalPoints[2];
    goalPoints[0].translation.x() = 4500.;
    goalPoints[1].translation.x() = 4500.;
    goalPoints[0].translation.y() = 730.;
    goalPoints[1].translation.y() = -730.;

    std::vector<float> leftPoints;
    std::vector<float> rightPoints;
    std::vector<float> freeAreas;

    float div1,div2,div3;
    if((goalPoints[0].translation.x() - myPose.translation.x() ) == 0){
        div1 = 0.1;
    }else{
        div1 = (goalPoints[0].translation.x() - myPose.translation.x() );
    }

    // y = ((4500 - x1)/(x2 - x1))*(y2 -y1) + y1

    float firstM = (goalPoints[0].translation.y() - myPose.translation.y() )/div1;
    float secondM = (goalPoints[1].translation.y() - myPose.translation.y() )/div2;
    int i, k;
    Obstacle swapper;




    for(i = 0; i < opponents.size(); i++){
        for(k = 0; k < opponents.size(); k++){
            if(opponents.at(i).left.y() > opponents.at(k).left.y() ){
                swapper = opponents.at(k);
                opponents.at(k) = opponents.at(i);
                opponents.at(i) = swapper;
            }
        }
    }
    for(i = 0; i < opponents.size(); i++){
        if((opponents.at(i).left.x() - myPose.translation.x()) == 0){
            div1 = 0.1;
        }else{
            div1 = opponents.at(i).left.x() - myPose.translation.x();
        }

        float y = ((4500 - myPose.translation.x())/div1)*(opponents.at(i).left.y() - myPose.translation.y()) + myPose.translation.y();
        leftPoints.push_back(y);
    }

    for(i = 0; i < opponents.size(); i++){
        if((opponents.at(i).right.x() - myPose.translation.x()) == 0){
            div1 = 0.1;
        }else{
            div1 = opponents.at(i).right.x() - myPose.translation.x();
        }

        float y = ((4500 - myPose.translation.x())/div1)*(opponents.at(i).right.y() - myPose.translation.y()) + myPose.translation.y();
        rightPoints.push_back(y);
    }

    //gestisco caso in cui tutti gli obstacle sono fuori dalla porta
    bool noneInside = true;
    float begin = 730.;
    float end = -730.;

    for(i = 0; i < leftPoints.size(); i++){
        //caso 1
        if((leftPoints.at(i) < 730. && leftPoints.at(i) > -730.) || (rightPoints.at(i) < 730. && rightPoints.at(i) > -730.)){
            noneInside = false;
        }
        if(leftPoints.at(i) > 730. && rightPoints.at(i) < 730.){
            begin = rightPoints.at(i);

        }
        if(leftPoints.at(i) > -730. && rightPoints.at(i) < -730.){
            end = leftPoints.at(i);

        }
    }
    if(noneInside == true){
        freeAreas.push_back(begin);
        freeAreas.push_back(end);
        return freeAreas;
    }
    freeAreas.push_back(begin);
    for(i = 0; i < leftPoints.size(); i++){
        if(leftPoints.at(i) < begin && leftPoints.at(i) > end){
            freeAreas.push_back(leftPoints.at(i));
        }
        if(rightPoints.at(i) > end && rightPoints.at(i) < begin){
            freeAreas.push_back(rightPoints.at(i));
        }
    }
    freeAreas.push_back(end);



    return freeAreas;
};

  libCodeRelease.computeTarget = [&](std::vector<float> freeAreas) -> std::vector<float> {
    float interval = 0;
    float maxInterval = 0;
    float maxIndex = 0;
    int i;
    if(freeAreas.size()%2 == 0){
        for(i = 0; i < freeAreas.size() -1; i+=2){
            interval = abs(freeAreas.at(i) -  freeAreas.at(i+1));
            if(maxInterval < interval){
                maxInterval = interval;
                maxIndex = i;
            }
        }
    }else{
        for(i = 1; i < freeAreas.size() -1; i+=2){
            interval = abs(freeAreas.at(i) -  freeAreas.at(i+1));
            if(maxInterval < interval){
                maxInterval = interval;
                maxIndex = i;
            }
        }
    }

    std::vector<float> datas;
    datas.push_back((freeAreas.at(maxIndex) + freeAreas.at(maxIndex +1))/2);
    float freePercentage = abs((freeAreas.at(maxIndex) - freeAreas.at(maxIndex +1)))/14.6;
    datas.push_back(freePercentage);
    return datas;
};

libCodeRelease.computeBetterTarget = [&](float target, Pose2f robotPose) -> Pose2f {
    float div1;



    double m = (double)(target/4500.f);
    //double x = robotPose.translation.x() + 1900;
    double x = 4500.f;
    double y = (double)(m*x);

    //std::cout<<"target "<<target<<" m ="<<m<<" x "<<x<<" y "<<y<<std::endl;
    Pose2f betterTarget;
    betterTarget.translation.x() = x;
    betterTarget.translation.y() = y;
    //std::cout<<" bt x "<<betterTarget.translation.x()<<" bt y "<<betterTarget.translation.y()<<std::endl;
    return betterTarget;
};

libCodeRelease.approachPoint = [&](Pose2f target, Pose2f globBall, bool behind) -> Pose2f {
    Pose2f approachP;
    if(behind){
        approachP.translation.x() = globBall.translation.x() - 200;
    }else{
        approachP.translation.x() = globBall.translation.x() + 200;
    }
    approachP.translation.y() = (((approachP.translation.x() - globBall.translation.x())/target.translation.x()-globBall.translation.x())*
        (globBall.translation.y()-target.translation.y())) + globBall.translation.y(); 
    return approachP;
};

libCodeRelease.getReadyPose = [&](bool kickoff, Role::RoleType rRole) ->Pose2f {
    switch(rRole){
        case 6:
            rRole = Role::RoleType::defender;
            break;
        case 7:
            rRole = Role::RoleType::supporter;
            break;
        case 8:
            rRole = Role::RoleType::striker;
            break;
        case 9:
            rRole = Role::RoleType::jolly;
            break;
    }
    if( rRole == Role::RoleType::goalie )
        return glob2Rel(SPQR::GOALIE_BASE_POSITION_X, SPQR::GOALIE_BASE_POSITION_Y);
    else if( rRole == Role::RoleType::striker )
    {
        if(kickoff)
            return glob2Rel(SPQR::STRIKER_KICKOFF_POSITION_X, SPQR::STRIKER_KICKOFF_POSITION_Y);
        else
            return glob2Rel(SPQR::STRIKER_NO_KICKOFF_POSITION_X, SPQR::STRIKER_NO_KICKOFF_POSITION_Y);
    }
    else if( rRole == Role::RoleType::defender )
            return glob2Rel(libCodeRelease.defenderPosition.x(), libCodeRelease.defenderPosition.y());
    else if( rRole == Role::RoleType::supporter )
            return glob2Rel(libCodeRelease.supporterPosition.x(), libCodeRelease.supporterPosition.y());

    else if( rRole == Role::RoleType::jolly )
        return glob2Rel(SPQR::JOLLY_DEFAULT_POSITION_X, SPQR::JOLLY_DEFAULT_POSITION_Y);
    else
        return glob2Rel(.0f, SPQR::FIELD_DIMENSION_Y);
};










 libCodeRelease.getTriang = [&] (Pose2f pa, Pose2f pb, Pose2f pc) -> Vector3f
 {
  return getTriang3Points(pa, pb, pc);
 };







  libCodeRelease.radiansToDegree = [&](float x) -> float
  {
    return (x*180)/3.14159265358979323846;
  };

  //ADDED
  //TODO insert the position on the field
  libCodeRelease.absBallPosition = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
  globBallY = libCodeRelease.absBallPosition.translation.y();
  libCodeRelease.defenderPosition = updateDefender();
  libCodeRelease.supporterPosition = updateSupporter();
  libCodeRelease.goaliePosition = updateGoalie();
  libCodeRelease.jollyPosition = Vector2f(800, -800);
  libCodeRelease.angleForDefender = angleToTarget(libCodeRelease.defenderPosition.x(), libCodeRelease.defenderPosition.y());
  libCodeRelease.angleForSupporter = angleToTarget(libCodeRelease.supporterPosition.x(), libCodeRelease.supporterPosition.y());
  libCodeRelease.angleForJolly = angleToTarget(800, -800);
  libCodeRelease.searcer_1Position = Vector2f(2800, 1500);
  libCodeRelease.searcer_2Position = Vector2f(2800, -1500);
  libCodeRelease.searcer_3Position = Vector2f(-2800, 1500);
  libCodeRelease.searcer_4Position = Vector2f(-2800, -1500);

}

float LibCodeReleaseProvider::angleToTarget(float x, float y)
{
    //gets the relative position of the point to go for the robot
    Pose2f relativePosition = glob2Rel(x,y);
    //std::cerr << "y relativa: "<< relativePosition.translation.y() << ", x relativa: "<<relativePosition.translation.x() << std::endl;
    //return radiansToDegree(atan2f(relativePosition.translation.y(), relativePosition.translation.x()));
    return (atan2f(relativePosition.translation.y(), relativePosition.translation.x()));

    //return glob2Rel(x, y).translation.angle();
}

Pose2f LibCodeReleaseProvider::glob2Rel(float x, float y)
{
    Vector2f result;
    float theta = 0;
    float tempX = x - theRobotPose.translation.x();
    float tempY = y - theRobotPose.translation.y();

    result.x() = tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation);
    result.y() = -tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation);

    return Pose2f(theta /*deg*/, result.x(),result.y());
}

//returns the degree value of an angle
float LibCodeReleaseProvider::radiansToDegree(float x)
{
    return (x*180)/3.14159265358979323846;
}

Vector2f LibCodeReleaseProvider::updateDefender()
{
    Pose2f globBall;
    if ( theFrameInfo.time - theBallModel.timeWhenLastSeen < 1000)
        globBall = rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
    else
        globBall = Pose2f(theTeamBallModel.position.x(),theTeamBallModel.position.y());

    int sign = globBall.translation.y()/std::abs(globBall.translation.y());
//    return Vector2f(std::min(std::max(-3800.f, -4500.f+(globBall.translation.x()+4500.f)*0.5f), -3200.f)-350.f,
//                    ((globBall.translation.y()+sign*900.f)*0.6f));
    return Vector2f(-3400.f, -700.f);
}

Vector2f LibCodeReleaseProvider::updateSupporter(){

    Pose2f globBall;
    if ( theFrameInfo.time - theBallModel.timeWhenLastSeen < 1000)
        globBall = rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
    else
        globBall = Pose2f(theTeamBallModel.position.x(),theTeamBallModel.position.y());

    int sign = globBall.translation.y()/std::abs(globBall.translation.y());
//    return Vector2f(std::min(std::max(-3800.f, -4500.f+(globBall.translation.x()+4500.f)*0.5f), -3200.f)+50.f,
//                    ((globBall.translation.y()-sign*900.f)*0.6f));
    return Vector2f(-2900.f, +700.f);

}

Vector2f LibCodeReleaseProvider::updateGoalie()
{
    Pose2f globBall = rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());

                    //-5055
    float deltaX = ((theFieldDimensions.xPosOwnGoal - globBall.translation.x()));// * 0.5f);
    float deltaY = ((globBall.translation.y()));// * 0.5f);

         if(deltaX > theFieldDimensions.xPosOwnPenaltyArea)
             deltaX = theFieldDimensions.xPosOwnPenaltyArea - 200 ;

         if (deltaY < theFieldDimensions.yPosRightGoal + 200 )
             deltaY = theFieldDimensions.yPosRightGoal + 300;

         if (deltaY > theFieldDimensions.yPosLeftGoal - 200 )
             deltaY = theFieldDimensions.yPosLeftGoal - 300;

         else if(deltaY < -200 && deltaY > theFieldDimensions.yPosRightGoal + 200 || deltaY > 200 && deltaY < theFieldDimensions.yPosRightGoal - 200 )
            deltaY = deltaY/2;


        //     std::cout << "x: " << deltaX << std::endl;
        //     std::cout << "y: " << deltaY << std::endl;
    //std::cout<< "LIBCODERELEASE" << deltaX << "  "<< deltaY<<std::endl;
     return Vector2f(deltaX, deltaY);
}


Pose2f LibCodeReleaseProvider::rel2Glob(float x, float y)
{
    Vector2f result;
    float rho = sqrt((x * x) + (y * y));

    result.x() = theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x)));
    result.y() = theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x)));

    return Pose2f(result.x(),result.y());
}

bool LibCodeReleaseProvider::isGoalieInStartingPosition()
{

   if( isValueBalanced(theRobotPose.translation.x(), SPQR::GOALIE_BASE_POSITION_X+1000, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
            isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y+1000, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
        return true;
    else
        return false;
}

bool LibCodeReleaseProvider::isValueBalanced(float currentValue, float target, float bound)
{
    float minErr = currentValue - (target - bound);
    float maxErr = currentValue - (target + bound);

    if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
        return true;
    else
        return false;
}

bool LibCodeReleaseProvider::isBallInKickAwayRange()
{
    if( theBallModel.estimate.position.norm() < SPQR::GOALIE_KICK_AWAY_RANGE )
        return true;
    else
        return false;
}

float LibCodeReleaseProvider::distance(Pose2f p1, Pose2f p2){
    float diffX = p1.translation.x() - p2.translation.x();
    float diffY = p1.translation.y() - p2.translation.y();

    return sqrt((diffX*diffX)+(diffY*diffY));
}

bool LibCodeReleaseProvider::isBallInArea()
{
    Pose2f gloBall = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
    if (between(gloBall.translation.x(), -4500, -3900) && between(gloBall.translation.y(), -1100, 1100))
        return true;
    else
        return false;
}

bool LibCodeReleaseProvider::between(float value, float min, float max)
{
    return value >= min && value <= max;
}

Pose2f LibCodeReleaseProvider::getGoalieCoverPosition()
{
    // Decide if to use the global ball or the DWK one
    //~ Pose2f pos;
    //~ if (libCodeRelease.timeSinceBallWasSeen() < 3000)
        //~ pos = Pose2f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
    //~ else if (theSpqrDWKcombiner.timeSinceWasSeen < 6000)
        //~ pos = Pose2f(theSpqrDWKcombiner.estimated_ball_relative.x(), theSpqrDWKcombiner.estimated_ball_relative.y());
    //~ else
        //~ pos = Pose2f(8000, 8000); // a big number to have norm outside the range
    //~ float normPos = norm(pos.translation.x(), pos.translation.y());
    //~
    //~ if (normPos < SPQR::GOALIE_KICK_AWAY_RANGE*3) {
        //~ return Pose2f(SPQR::GOALIE_BASE_POSITION_X, pos.translation.y() / 3000 * 650);
    //~ } else
        return Pose2f(SPQR::GOALIE_BASE_POSITION_X, SPQR::GOALIE_BASE_POSITION_Y);
}

bool LibCodeReleaseProvider::isGoalieInAngle()
{
    //~ if(isBallInCoverRange())
        //~ if (between(angleToTarget(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()),
                //~ Angle::fromDegrees(-10.f),
                //~ Angle::fromDegrees(10.f) ) )
            //~ return true;
        //~ else
            //~ return false;
    //~ else
        if (between(theRobotPose.rotation, Angle::fromDegrees(-10.f), Angle::fromDegrees(10.f) ))
            return true;
        else
            return false;
}

// AreaX is between -4500 and -3900, areaY is between -1100 and 1100
bool LibCodeReleaseProvider::isGoalieInArea()
{
    if (between(theRobotPose.translation.x(), -4500, -3900) && between(theRobotPose.translation.y(), -1100, 1100))
        return true;
    else
        return false;
}

float LibCodeReleaseProvider::norm(float x, float y)
{
    return sqrt((x*x) + (y*y));
}

float LibCodeReleaseProvider::getGoalieCoverAngleDisplacement()
{
    //~ if (isBallInCoverRange())
        //~ return -angleToTarget(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
    //~ else
        return theRobotPose.rotation;
}

bool LibCodeReleaseProvider::isGoalieInKickAwayRange()
{
    Pose2f gloBall = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
    if (between(gloBall.translation.x(), -4500, -3800) && between(gloBall.translation.y(), -1200, 1200))
        return true;
    else
        return false;
}

Pose2f LibCodeReleaseProvider::disambiguateCell(int cell){
  Pose2f myPose;
  int myY = (int)(cell/18);
  int myX = (int)(cell%18);
  myPose.translation.x() = ((500.f * myX) + 250.) -4500.;
  myPose.translation.y() = ((500.f * myY) + 250.) -3000.;

  return myPose;
}

int LibCodeReleaseProvider::discretizePose(float x, float y){

    int myX = (int)((x + 4500.)/500.);
    int myY = (int)((y + 3000.)/500.);
    int discretizedPose;
    if(myX %18 == 0 && myX != 0){
      discretizedPose = myX + 18*myY -1;
    }else{
      discretizedPose = myX + 18*myY;
    }

    return discretizedPose;
}





















//////////////////////ROBA NUOVA


bool LibCodeReleaseProvider::canPass(Pose2f targetPose, Pose2f shootingPose, std::vector<Obstacle> opponents)
{
  float m = 0;
  bool sameX = false;
  bool sameY = false;
  //se hanno la stessa x (più o meno)
  if(std::abs(shootingPose.translation.x() - targetPose.translation.x()) <= 0.5){
    sameX = true;
  }
  else{
    float m = shootingPose.translation.y() - targetPose.translation.y();
    if(std::abs(m) <= 0.5 ){
      sameY = true;
    }
    else{
      m /= shootingPose.translation.x() - targetPose.translation.x();
    }
  }
  float q = shootingPose.translation.y() - m * shootingPose.translation.x();
  float qThreshold = 100.f;
  //TODO inserire il vettore degli opponents
  for(auto const& opponent : opponents){
    if(sameX){
      if(std::abs(opponent.center.x() - shootingPose.translation.x()) < qThreshold){
        return false;
      }
    }
    else if(sameY){
      if(std::abs(opponent.center.y() - shootingPose.translation.y()) < qThreshold){
        return false;
      }
    }
    else{
      for(int i = -50; i < 50; i++){
        //se l'opponent si trova su una retta del fascio
        if((opponent.center.y() - m * opponent.center.x() - q - (float) i * 10.f) <= 0.5f ){
          return false;
        }
      }
    }
  }
  return true;
}



bool LibCodeReleaseProvider::otherStateHasBall(int myPose, Vector2f ballPose){

    int ballCell=  discretizePose(ballPose.x(),ballPose.y());

    if(myPose == ballCell){
        return true;
    }else{
        return false;
    }
}


bool LibCodeReleaseProvider::freeBallSight(Pose2f myPose, Vector2f ballPose, std::vector<Obstacle> opponents){

  float ballDist = norm(myPose.translation.x() - ballPose.x(), myPose.translation.y() - ballPose.y());
  int i;
  float oppDist;
  bool sameSide;


  for(i = 0; i < opponents.size(); i++){

    if((myPose.translation.x() - ballPose.x()) >= 0 && (myPose.translation.x() - opponents.at(i).center.x()) >= 0){
      sameSide = true;
    }else if((myPose.translation.x() - ballPose.x()) <= 0 && (myPose.translation.x() - opponents.at(i).center.x()) <= 0){
      sameSide = true;
    }else{
      sameSide = false;
    }
    oppDist = norm(myPose.translation.x() - opponents.at(i).center.x(),
      myPose.translation.y() - opponents.at(i).center.y());

    if(oppDist < ballDist && sameSide == true){

      float divX = myPose.translation.x() - ballPose.x();
      if(divX == 0){
        divX = 0.1f;
      }

      float divY = myPose.translation.y() - ballPose.y();
      if(divY == 0){
        divY = 0.1f;
      }

      float numX =  myPose.translation.x() - opponents.at(i).center.x();
      float numY = myPose.translation.y() - opponents.at(i).center.y();

      float rightEqside = (numY/divY) * divX;

      //std::cout<<"numX "<<numX<<" rightEqside"<<rightEqside<<std::endl;
      if(numX > rightEqside -350 && numX < rightEqside + 350){
        return false;
      }
    }
  }
  return true;
}

bool LibCodeReleaseProvider::stateHasBall(Pose2f myPose, Vector2f ballPose){

    int myCell =  discretizePose(myPose.translation.x(),myPose.translation.y());
    int ballCell=  discretizePose(ballPose.x(),ballPose.y());

    if(myCell == ballCell || theBallModel.estimate.position.norm() < 500.f ){
        return true;
    }else{
        return false;
    }
}

std::vector<Obstacle> LibCodeReleaseProvider::globalizeObstacles(Pose2f myPose, std::vector<Obstacle> oldOpponents){

    int i;
    std::vector<Obstacle> opponents;
    for(i = 0; i < oldOpponents.size(); i++){
        Pose2f newLeft =  rel2Glob(oldOpponents.at(i).left.x(),oldOpponents.at(i).left.y());
        Pose2f newCenter =  rel2Glob(oldOpponents.at(i).center.x(),oldOpponents.at(i).center.y());
        Pose2f newRight =  rel2Glob(oldOpponents.at(i).right.x(),oldOpponents.at(i).right.y());
        oldOpponents.at(i).left.x() = newLeft.translation.x();
        oldOpponents.at(i).left.y() = newLeft.translation.y();

        oldOpponents.at(i).center.x() = newCenter.translation.x();
        oldOpponents.at(i).center.y() = newCenter.translation.y();

        oldOpponents.at(i).right.x() = newRight.translation.x();
        oldOpponents.at(i).right.y() = newRight.translation.y();
        if(oldOpponents.at(i).center.x() >  myPose.translation.x()){

            opponents.push_back(oldOpponents.at(i));
        }

    }

    return opponents;
}

std::vector<float> LibCodeReleaseProvider::computeFreeAreas(Pose2f myPose, std::vector<Obstacle> opponents){
    Pose2f goalPoints[2];
    goalPoints[0].translation.x() = 4500.;
    goalPoints[1].translation.x() = 4500.;
    goalPoints[0].translation.y() = 730.;
    goalPoints[1].translation.y() = -730.;

    std::vector<float> leftPoints;
    std::vector<float> rightPoints;
    std::vector<float> freeAreas;

    float div1,div2,div3;
    if((goalPoints[0].translation.x() - myPose.translation.x() ) == 0){
        div1 = 0.1;
    }else{
        div1 = (goalPoints[0].translation.x() - myPose.translation.x() );
    }

    // y = ((4500 - x1)/(x2 - x1))*(y2 -y1) + y1

    float firstM = (goalPoints[0].translation.y() - myPose.translation.y() )/div1;
    float secondM = (goalPoints[1].translation.y() - myPose.translation.y() )/div2;
    int i, k;
    Obstacle swapper;




    for(i = 0; i < opponents.size(); i++){
        for(k = 0; k < opponents.size(); k++){
            if(opponents.at(i).left.y() > opponents.at(k).left.y() ){
                swapper = opponents.at(k);
                opponents.at(k) = opponents.at(i);
                opponents.at(i) = swapper;
            }
        }
    }
    for(i = 0; i < opponents.size(); i++){
        if((opponents.at(i).left.x() - myPose.translation.x()) == 0){
            div1 = 0.1;
        }else{
            div1 = opponents.at(i).left.x() - myPose.translation.x();
        }

        float y = ((4500 - myPose.translation.x())/div1)*(opponents.at(i).left.y() - myPose.translation.y()) + myPose.translation.y();
        leftPoints.push_back(y);
    }

    for(i = 0; i < opponents.size(); i++){
        if((opponents.at(i).right.x() - myPose.translation.x()) == 0){
            div1 = 0.1;
        }else{
            div1 = opponents.at(i).right.x() - myPose.translation.x();
        }

        float y = ((4500 - myPose.translation.x())/div1)*(opponents.at(i).right.y() - myPose.translation.y()) + myPose.translation.y();
        rightPoints.push_back(y);
    }

    //gestisco caso in cui tutti gli obstacle sono fuori dalla porta
    bool noneInside = true;
    float begin = 730.;
    float end = -730.;

    for(i = 0; i < leftPoints.size(); i++){
        //caso 1
        if((leftPoints.at(i) < 730. && leftPoints.at(i) > -730.) || (rightPoints.at(i) < 730. && rightPoints.at(i) > -730.)){
            noneInside = false;
        }
        if(leftPoints.at(i) > 730. && rightPoints.at(i) < 730.){
            begin = rightPoints.at(i);

        }
        if(leftPoints.at(i) > -730. && rightPoints.at(i) < -730.){
            end = leftPoints.at(i);

        }
    }
    if(noneInside == true){
        freeAreas.push_back(begin);
        freeAreas.push_back(end);
        return freeAreas;
    }
    freeAreas.push_back(begin);
    for(i = 0; i < leftPoints.size(); i++){
        if(leftPoints.at(i) < begin && leftPoints.at(i) > end){
            freeAreas.push_back(leftPoints.at(i));
        }
        if(rightPoints.at(i) > end && rightPoints.at(i) < begin){
            freeAreas.push_back(rightPoints.at(i));
        }
    }
    freeAreas.push_back(end);



    return freeAreas;
}

std::vector<float> LibCodeReleaseProvider::computeTarget(std::vector<float> freeAreas){
    float interval = 0;
    float maxInterval = 0;
    float maxIndex = 0;
    int i;
    if(freeAreas.size()%2 == 0){
        for(i = 0; i < freeAreas.size() -1; i+=2){
            interval = abs(freeAreas.at(i) -  freeAreas.at(i+1));
            if(maxInterval < interval){
                maxInterval = interval;
                maxIndex = i;
            }
        }
    }else{
        for(i = 1; i < freeAreas.size() -1; i+=2){
            interval = abs(freeAreas.at(i) -  freeAreas.at(i+1));
            if(maxInterval < interval){
                maxInterval = interval;
                maxIndex = i;
            }
        }
    }

    std::vector<float> datas;
    datas.push_back((freeAreas.at(maxIndex) + freeAreas.at(maxIndex +1))/2);
    float freePercentage = abs((freeAreas.at(maxIndex) - freeAreas.at(maxIndex +1)))/14.6;
    datas.push_back(freePercentage);
    return datas;
}

Pose2f LibCodeReleaseProvider::computeBetterTarget(float target, Pose2f robotPose){
    float div1;



    double m = (double)(target/4500.f);
    //double x = robotPose.translation.x() + 1900;
    double x = 4500.f;
    double y = (double)(m*x);

    //std::cout<<"target "<<target<<" m ="<<m<<" x "<<x<<" y "<<y<<std::endl;
    Pose2f betterTarget;
    betterTarget.translation.x() = x;
    betterTarget.translation.y() = y;
    //std::cout<<" bt x "<<betterTarget.translation.x()<<" bt y "<<betterTarget.translation.y()<<std::endl;
    return betterTarget;
}

Pose2f LibCodeReleaseProvider::approachPoint(Pose2f target, Pose2f globBall, bool behind){
    Pose2f approachP;
    if(behind){
        approachP.translation.x() = globBall.translation.x() - 200;
    }else{
        approachP.translation.x() = globBall.translation.x() + 200;
    }
    approachP.translation.y() = (((approachP.translation.x() - globBall.translation.x())/target.translation.x()-globBall.translation.x())*
        (globBall.translation.y()-target.translation.y())) + globBall.translation.y(); 
    return approachP;
}

Pose2f LibCodeReleaseProvider::getReadyPose(bool kickoff, Role::RoleType rRole)
{
    if( rRole == Role::RoleType::goalie )
        return glob2Rel(SPQR::GOALIE_BASE_POSITION_X, SPQR::GOALIE_BASE_POSITION_Y);
    else if( rRole == Role::RoleType::striker )
    {
//        if(kickoff) return glob2Rel(SPQR::STRIKER_KICKOFF_POSITION_X, SPQR::STRIKER_KICKOFF_POSITION_Y);
//        else
                return glob2Rel(SPQR::STRIKER_NO_KICKOFF_POSITION_X, SPQR::STRIKER_NO_KICKOFF_POSITION_Y);
    }
//    else if( rRole == Role::RoleType::defender )
//        return glob2Rel(SPQR::DEFENDER_DEFAULT_POSITION_X, SPQR::DEFENDER_DEFAULT_POSITION_Y);
//    else if( rRole == Role::RoleType::supporter )
//        return glob2Rel(SPQR::SUPPORTER_DEFAULT_POSITION_X, SPQR::SUPPORTER_DEFAULT_POSITION_Y);
    else if( rRole == Role::RoleType::defender )
        return glob2Rel(updateDefender().x(), updateDefender().y());
    else if( rRole == Role::RoleType::supporter )
        return glob2Rel(updateSupporter().x(), updateSupporter().y());

    else if( rRole == Role::RoleType::jolly )
        return glob2Rel(SPQR::JOLLY_DEFAULT_POSITION_X, SPQR::JOLLY_DEFAULT_POSITION_Y);
    else
        return glob2Rel(.0f, SPQR::FIELD_DIMENSION_Y);
}

Vector3f LibCodeReleaseProvider::getTriang3Points(Pose2f pose_a, Pose2f pose_b, Pose2f pose_c)
{
  Vector3f results;
  Vector2f pa, pb ,pc; 
  pa << pose_a.translation.x(),pose_a.translation.y();
  pb << pose_b.translation.x(),pose_b.translation.y();
  pc << pose_c.translation.x(),pose_c.translation.y();
  float a = (pa-pb).norm();
  float b = (pb-pc).norm();
  float c = (pc-pa).norm();


  results << atan2f(a,c), atan2f(a,b), atan2f(b,c);
  return results;

}
