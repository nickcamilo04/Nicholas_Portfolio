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
{
    status_ = pfms::PlatformStatus::RUNNING;
    threads_.push_back(std::thread(&Ackerman::audiZooms, this));
    Audi audi;
}

Ackerman::~Ackerman(){
    running_ = false;
    //joining the threads
    for (auto & t: threads_){
        t.join();
    }
}

void Ackerman::run(void){
    running_ = true;
    std::unique_lock<std::mutex> lck(mtxStart_);
    mtxStart_.unlock();
    cvStart_.notify_all();
    pfmsConnectorPtr_->send(status_);
}


pfms::PlatformStatus Ackerman::status(void){
     return status_;
}


bool Ackerman::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    getOdometry();
    goalUpdate_ = -1;
    pfms::nav_msgs::Odometry newOrigin = currentMovement;
    goalVec_.clear();
    
    // Iterate over each goal in the vector 
    for (auto& goal : goals) {
        goalUpdate_++;
        goalVec_.push_back(goal);

        goal_.point = goal;  // Assign the current goal to the point member variable of the goal_ object
        pfmsConnectorPtr_->send(goal_);  // Send the goal_ object through a pointer named pfmsConnectorPtr_
        
        // Read data into the origin_ object using the read method of the object pointed to by pfmsConnectorPtr_
        pfmsConnectorPtr_->read(currentMovement, getPlatformType());

        // Call method from audi library to compute steering and distance
        bool yep = audi.checkOriginToDestination(newOrigin, goalVec_.at(goalUpdate_), distance_, time_, estimatedGoalPose_);

        newOrigin = currentMovement;

        // Check if the steering computation was successful
        if (!yep) {
            return false;  // Return false if any goal's steering computation fails
        }
    }

    return true;  // Return true if all goals are set successfully                                      
}


bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                    pfms::geometry_msgs::Point goal,
                                    double& distance,
                                    double& time,
                                    pfms::nav_msgs::Odometry& estimatedGoalPose){
return audi.checkOriginToDestination(currentMovement, goal_.point, distance_, time_, estimatedGoalPose);                              

}


pfms::PlatformType Ackerman::getPlatformType(void){
    return platformType_;
}


double Ackerman::distanceToGoal(void){
    
    audi.checkOriginToDestination(currentMovement, goal_.point, distance_, time_, estimatedGoalPose_);
    audi.computeSteering(currentMovement, goal_.point, steering_, distance_);

    return distance_;   //return distance computed by audi function
}


double Ackerman::timeToGoal(void){

    audi.checkOriginToDestination(currentMovement, goal_.point, distance_,time_, estimatedGoalPose_);
    return time_;           // return time computed by audi function
}


pfms::nav_msgs::Odometry Ackerman::getOdometry(void){
    return currentMovement;
}


std::vector<pfms::geometry_msgs::Point> Ackerman::getObstacles(void){

}

bool Ackerman::audiZooms(){
    std::unique_lock<std::mutex> lck(mtxStart_);
    cvStart_.wait(lck, [&](){return running_ == true;});

    
while(running_ == true) {
    
    bool OK = pfmsConnectorPtr_->read(currentMovement, getPlatformType());                //Reads data from pfmsConnectorPtr_ into currentMovement and checks if the operation is successful, storing the result in OK.
    double xDifference = abs(goal_.point.x-currentMovement.position.x);                   //Determines the differences in x position for goal and current point 
    double yDifference = abs(currentMovement.position.y-goal_.point.y);    
    absVelocity_ = sqrt((pow(currentMovement.linear.x,2)) + pow(currentMovement.linear.y,2));               //Determines the differences in y position for goal and current point 
    unsigned long iterator = 0;
    setTolerance(0.5); //for now


    status_ = pfms::PlatformStatus::RUNNING;
    pfmsConnectorPtr_->send(status_);

for (unsigned int i = 0; i < goalVec_.size(); i++) {
    goal_.point = goalVec_.at(i);

    while(distanceToGoal() > Controller::toleranceError_) 
    {
        iterator++;                                                                           //This will count how many times its in the loop 
        pfms::commands:: Ackerman cmd {iterator, brake_, steering_, throttle_};                  //Command which is the packet of info sent to the simulation, got from snippets files

        pfmsConnectorPtr_->send(cmd);  

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if(origin_.yaw == steering_)
        {
            steering_;
        }

        if (distanceToGoal() <= 3* Controller::toleranceError_)      //3 times the tollerance (0.5) seems to be the sweet spot
        { 
            brake_ = 1000; //(1000 * 1 / distanceToGoal()) - (1000*(1/absVelocity_)*1.5);                                //Calculates a braking value based on the inverse of the distance to the goal.
        } else {
            brake_ = 0;
        }

    OK = pfmsConnectorPtr_->read(currentMovement,getPlatformType());         //Reads data into currentMovement from pfmsConnectorPtr_ and updates OK.
            }
            

        }
        status_ = pfms::PlatformStatus::IDLE;
            pfmsConnectorPtr_->send(status_);

            if (!running_){
                break;
            }

            std::this_thread::sleep_for (std::chrono::milliseconds(10));
            break;
    }
    return true;
}
