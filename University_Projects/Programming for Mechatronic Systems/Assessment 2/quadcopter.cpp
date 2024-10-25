#include "quadcopter.h"

Quadcopter::Quadcopter(): 
platformType_(pfms::PlatformType::QUADCOPTER),
turn_l_r(0.0), // Initialize the left-right turn to 0
move_l_r(0.0), // Initialize the left-right movement to 0
move_u_d(0.0), // Initialize the up-down movement to 0
move_f_b(0.1), // Initialize the forward-backward movement to 0.1
speed_(1), // Initialize the speed to 1
actualTolerance_(0.5) // Initialize the tolerance to 0.5
{
    status_ = pfms::PlatformStatus::RUNNING; // Set the status of the Quadcopter to RUNNING
    threads_.push_back(std::thread(&Quadcopter::quadTakeoff, this)); // Create a thread for takeoff
    setTolerance(0.5); // Set the tolerance to 0.5
}

void Quadcopter::run(void){
    running_ = true; // Set the running flag to true
    std::unique_lock<std::mutex> lck(mtxStart_); // Acquire a lock for starting the thread
    mtxStart_.unlock(); // Unlock the mutex
    cvStart_.notify_all(); // Notify all threads about the start
    pfmsConnectorPtr_->send(status_); // Send the status through the connector
}

Quadcopter::~Quadcopter(){
    running_ = false; // Set the running flag to false
    //joining the threads
    for (auto & t: threads_){ // Loop through each thread
        t.join(); // Join the thred
    }
}

pfms::PlatformStatus Quadcopter::status(void){
    return status_; // Return the status of the Quadcopter
}

bool Quadcopter::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    goalVec_.clear();
    std::cout << "quad goals: " << sizeof(goals) << std::endl;
   
    unsigned long i = 0;

     goalUpdate_ = 0;
    
    // Iterate over each goal in the vector 
    for (const auto& goal : goals) {
        
        goalVec_.push_back(goal); // Add the goal to the vector

        pfmsConnectorPtr_->send({i, goalVec_.back()});  // Send the goal_ object through a pointer named pfmsConnectorPtr_
    
        i++;
    }
return true;
}


bool Quadcopter::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                    pfms::geometry_msgs::Point goal,
                                    double& distance,
                                    double& time,
                                    pfms::nav_msgs::Odometry& estimatedGoalPose){

}

pfms::PlatformType Quadcopter::getPlatformType(void){
    return platformType_;
}

double Quadcopter::distanceToGoal(void){

}

double Quadcopter::timeToGoal(void){

}

pfms::nav_msgs::Odometry Quadcopter::getOdometry(void){
    return currentMovement;
}

std::vector<pfms::geometry_msgs::Point> Quadcopter::getObstacles(void){

}

bool Quadcopter::quadTakeoff(){

    std::unique_lock<std::mutex> lck(mtxStart_); // Acquire a lock for starting the thread
    cvStart_.wait(lck, [&](){return running_ == true;});  // Wait until the thread starts
    goalUpdate_ = 0;
  while(running_ == true) { // Loop while the Quadcopter is running
    

    status_ = pfms::PlatformStatus::IDLE;
    pfmsConnectorPtr_->send(status_);  

    status_ = pfms::PlatformStatus::TAKEOFF; 
    pfmsConnectorPtr_->send(status_);
    std::cout << "Taking Off" << std::endl;

    unsigned long i = 0;
    int goalOrder = goalVec_.size() - 1; // Get the index of the last goal

    status_ = pfms::PlatformStatus::RUNNING;
    pfmsConnectorPtr_->send(status_);

    while(!goalsReached_) {
        std::cout << "Going to goal: " <<  unsigned(goalUpdate_) << ", out of: " << sizeof(goalVec_	) << std::endl;
        std::cout << ".................Running............" << std::endl;

        pfmsConnectorPtr_->read(currentMovement, getPlatformType()); // Read the current movement

        double dx = goalVec_.at(goalUpdate_).x - currentMovement.position.x; // Calculate the x difference
        double dy = goalVec_.at(goalUpdate_).y - currentMovement.position.y; // Calculate the y difference
        double dz = goalVec_.at(goalUpdate_).z - currentMovement.position.z; // Calculate the z difference

        std::cout << "dx: " << dx <<" ,dy: " << dy <<" ,dz: " << dz << ",tolerance: " << toleranceError_ << std::endl;

        if(std::abs(dy) <= toleranceError_ && std::abs(dx) <= toleranceError_ && goalUpdate_ == goalOrder){ // Check if the goal is reached
            goalsReached_ = true; //when all goals have been reached
            std::cout << "We reached all goals!" << std::endl;
        }

        if (std::abs(dy) <= toleranceError_ && std::abs(dx) <= toleranceError_) { // Check if the x and y differences are within tolerance
            goalUpdate_++; // Move to the next goal
            std::cout<<"Moving to goal: "<< goalUpdate_ << "\n";
        } 
        else {

            // If not all dimensions are within tolerance, continue moving towards the goal
            move_u_d = speed_ * (2.0 - currentMovement.position.z); // Calcuate the up-down movement
            move_l_r = speed_ * std::copysign(1.0, dy); // Adjust movement direction based on dy sign
            move_f_b = speed_ * std::copysign(1.0, dx); // Adjust movement direction based on dx sign
        }

        turn_l_r = 0; // Set the turn left-right to 0
        pfms::commands::Quadcopter cmd{i, turn_l_r, move_l_r, move_u_d, move_f_b};
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Send the command to the simulation
        pfmsConnectorPtr_->send(cmd); // Sending the package information to the simulation
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        i++;
    }

    std::cout<<"Landing: "<< goalUpdate_ << std::endl;
    status_ = pfms::PlatformStatus::LANDING;
    pfmsConnectorPtr_->send(status_);

    status_ = pfms::PlatformStatus::IDLE;
    pfmsConnectorPtr_->send(status_);

    if (!running_){
                break;
            }

            std::this_thread::sleep_for (std::chrono::milliseconds(50));
            break;
}
    return true;

}