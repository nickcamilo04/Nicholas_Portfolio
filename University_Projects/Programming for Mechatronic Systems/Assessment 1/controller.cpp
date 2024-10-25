#include "controller.h"
 
Controller::Controller(){
 pfms::nav_msgs::Odometry origin;
 pfms::geometry_msgs::Point goal;
 double distance = 0;
 double time = 0; 
 pfms::nav_msgs::Odometry& estimatedGoalPose();
 pfmsConnectorPtr_ = std::make_shared<PfmsConnector>();
 toleranceError_ = 0;
 time_ = 0.0;
}

bool Controller::setTolerance(double tolerance){

toleranceError_ = tolerance;
    return tolerance;

}

double Controller::timeInMotion(){

return time_;
}