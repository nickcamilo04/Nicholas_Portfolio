#include "ackerman.h"

using std::cout;
using std::endl; 


Ackerman::Ackerman():
    platformType_(pfms::PlatformType::ACKERMAN), //The platform we're using is Ackerman 
    initial_point_{-0.819,0}, //The initial point we will define as the origin 
    initialVelocity_{0,0}, //We will initialise our initial velocity as (0,0)
    current_point_{0,0,0}, //We will define our current position as (0,0) since upon creation our current and initial point are equal
    origin_{0, {0,0,0}, 0}, //We will define our odo type value as origin to have all our starting info (initialising seq, initial position, yaw, initial velocity)
    distance_(0.0), //We will have a variable to store the scalar value of how far we have travelled 
    brake_(0.0),throttle_(0.1),steering_(0.0), //We will initialise the values we use for our ackerman platform (brake = 0, throttle = 0, steering = 0 )
    estimatedGoalPose_{0,0,0,0}
{}

bool Ackerman::reachGoal(void){

    Audi audi; 
    bool reached_ = audi.computeSteering(origin_,goal_.point,steering_,distance_);  //trying a different method that will hopefully use the zoomZoom function to drive the car, where this function 
                                                                                    // just sets of flags to run it 
    if(reached_)
    {
       zoomZooms();
       return true;     
    } else {

      return false;
    }

}

bool Ackerman::setGoal(pfms::geometry_msgs::Point goal){
    Audi audi;
    goal_.point = goal;                                                //assigns the goal parameter to the point member variable of the goal_ object
    pfmsConnectorPtr_->send(goal_);                                    //sends the goal_ object through a pointer named pfmsConnectorPtr_
    pfmsConnectorPtr_->read(origin_,getPlatformType());                // data into the origin_ object using the read method of the object pointed to by pfmsConnectorPtr_
    currentMovement = origin_;                                         //assigns origin_ to currentMovement vriable

    bool yep =  audi.computeSteering(origin_, goal_.point, steering_, distance_);  //calling method from audi library and assigning to yep variable

return yep;                                                            //return bool output from computeSteering
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){
    

    Audi audi;
    return audi.checkOriginToDestination(currentMovement, goal_.point, distance_, time_, estimatedGoalPose); //using function from audi library
}

pfms::PlatformType Ackerman::getPlatformType(void){

return platformType_;     //return the suitable platform type
}

double Ackerman::distanceToGoal(void){
    Audi audi;
    checkOriginToDestination(origin_, goal_.point, distance_,time_, estimatedGoalPose_);

    return distance_;   //return distance computed by audi function
}

double Ackerman::timeToGoal(void){
    checkOriginToDestination(origin_, goal_.point, distance_,time_, estimatedGoalPose_);
return time_;           // return time computed by audi function
}


double Ackerman::distanceTravelled(void){

double xPointDiff = goal_.point.x - currentMovement.position.x;        //determined diff in x pos from goal to current point
double yPointDiff = goal_.point.y - currentMovement.position.y;        //determined diff in y pos from goal to current point
double distanceTravelled = sqrt(pow(xPointDiff, 2) + pow(yPointDiff, 2));       //get absolute distance through pythag

return distanceTravelled;
}

pfms::nav_msgs::Odometry Ackerman::getOdometry(void){

    return currentMovement;  //as simple as using the odometry given to us in the pipes library
}

bool Ackerman::zoomZooms(){

Audi audi;  
bool OK = pfmsConnectorPtr_->read(currentMovement, getPlatformType());                //Reads data from pfmsConnectorPtr_ into currentMovement and checks if the operation is successful, storing the result in OK.
double xDifference = abs(goal_.point.x-currentMovement.position.x);                   //Determines the differences in x position for goal and current point 
double yDifference = abs(currentMovement.position.y-goal_.point.y);                   //Determines the differences in y position for goal and current point 
unsigned long iterator = 0;
setTolerance(0.5);


while( distanceToGoal() > toleranceError_) 
{
    iterator++;                                                                           //This will count how many times its in the loop 
    pfms::commands:: Ackerman cmd {iterator,brake_,steering_,throttle_};                  //Command which is the packet of info sent to the simulation, got from snippets files

    pfmsConnectorPtr_->send(cmd);  

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if(origin_.yaw == steering_)
    {
    steering_;
    }

    if(distanceToGoal() <= 3*toleranceError_)      //3 times the tollerance (0.5) seems to be the sweet spot
    {
        brake_ = 1000 * 1 / distanceToGoal();                                //Calculates a braking value based on the inverse of the distance to the goal.
    } else {
    brake_ = 0;
    }

OK = pfmsConnectorPtr_->read(currentMovement,getPlatformType());         //Reads data into currentMovement from pfmsConnectorPtr_ and updates OK.
    
}

return true;    //only if goal is reached

}