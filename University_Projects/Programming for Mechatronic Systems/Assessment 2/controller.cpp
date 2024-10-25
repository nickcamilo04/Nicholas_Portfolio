#include "controller.h"
 
Controller::Controller(){
 pfms::nav_msgs::Odometry origin;
 pfms::geometry_msgs::Point goal;
 double distance = 0;
 double time = 0; 
 pfms::nav_msgs::Odometry& estimatedGoalPose();
 pfmsConnectorPtr_ = std::make_shared<PfmsConnector>();
 toleranceError_ = 0.5;
 time_ = 0.0;
}

bool Controller::setTolerance(double tolerance){

toleranceError_ = tolerance;
    return tolerance;
}

double Controller::timeTravelled(){
    return time_;
}

double Controller::distanceTravelled(){
    double xPointDiff = goal_.point.x - currentMovement_.position.x;        //determined diff in x pos from goal to current point
    double yPointDiff = goal_.point.y - currentMovement_.position.y;        //determined diff in y pos from goal to current point
    double distanceTravelled = sqrt(pow(xPointDiff, 2) + pow(yPointDiff, 2));       //get absolute distance through pythag

    return distanceTravelled;
}

void run(void){

}

pfms::PlatformStatus status(void){

}

bool setGoals(std::vector<pfms::geometry_msgs::Point> goals){

}

bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){

                                        }

pfms::PlatformType getPlatformType(void) {

}

double distanceToGoal(void){

}

double timeToGoal(void){

}

pfms::nav_msgs::Odometry getOdometry(void){

}

std::vector<pfms::geometry_msgs::Point> getObstacles(void){

}