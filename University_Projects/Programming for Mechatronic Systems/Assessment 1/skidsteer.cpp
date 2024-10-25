#include "skidsteer.h"

SkidSteer::SkidSteer() :
/*............................Initialising all our veriables, and having it so they can be updated and stored .....................................*/
   
    type_(pfms::PlatformType::SKIDSTEER), //The platform we're using is Skidsteer
    initial_point_{0,-5,0}, //The initial point we will define as the origin (0,-5,0)
    initialVelocity_{0,0}, //We will initialise our initial velocity as (0,0)
    current_point_{0,0,0}, //We will define our current position as (0,0) since upon creation our current and initial point are equal
    origin_{0, {0,0,0}, 0}, //We will define our odo type value as origin to have all our starting info (initialising seq, initial position, yaw, initial velocity)
    distance_(0.0), //We will have a variable to store the scalar value of how far we have travelled 
    goal_{0,1,-1,0}, //We will include a variable for our goal point as well 
    estimatedGoalPose_{0,0,0,0},
    absDist_(0.0),
    currentAngle_(0.0),
    XDist_(0.0),
    YDist_(0.0),
    goalAngle_(0.0),
    angleDifference_(0.0),
    angleError_(0.0)
{}

void SkidSteer::locationSet() {
       pfmsConnectorPtr_->read(currentMovement, getPlatformType());
       goalAngle_ = 0;
       XDist_ = (goal_.point.x-currentMovement.position.x); // calculating the differences between x coordinate and goal
       YDist_ = (goal_.point.y-currentMovement.position.y); // calculating the difference between y coordinate and goal 
       absDist_ = sqrt(pow(XDist_,2)+pow(YDist_,2));        // calculating the absolute dist using Euclidean formula
       currentAngle_ = currentMovement.yaw;

       double tanGoalAngle = abs(atan(YDist_/XDist_)); //calculate tangent of the goal angle

//the following are to clarify what quadrants the husky moves in, and then correct its movements accordingly

    if (XDist_ > 0 && YDist_ > 0) {
    goalAngle_ = tanGoalAngle; // top left
    }
    else if (XDist_ < 0 && YDist_ > 0) {
        goalAngle_ = M_PI - tanGoalAngle; // bottom left
    }
    else if (XDist_ > 0 && YDist_ < 0) {
        goalAngle_ = -tanGoalAngle; // top right
    }
    else if (XDist_ < 0 && YDist_ < 0) {
        goalAngle_ = -M_PI + tanGoalAngle; // bottom right
    }

//we need to account for when tan(0)
    else if (XDist_ == 0 && YDist_ > 0) {
        goalAngle_ = M_PI / 2; // top so +y
    }
    else if (XDist_ == 0 && YDist_ < 0) {
        goalAngle_ = -M_PI / 2; // bottom so -y
    }
    else if (XDist_ > 0 && YDist_ == 0) {
        goalAngle_ = 0; // right so +x
    }
    else if (XDist_ < 0 && YDist_ == 0) {
        goalAngle_ = M_PI; // left so -x
    }


    angleError_ = 5*(M_PI/180); //this should correspond to 0.08 degrees (will hardcode if not)
    angleDifference_ = abs(goalAngle_-currentAngle_);  //gets the difference between the angle of the goal and our current husky angle to determine when to move forward
       
    }

bool SkidSteer::reachGoal(void){

    int i = 0;

    pfmsConnectorPtr_->read(currentMovement, getPlatformType());

    while(absDist_ > toleranceError_) {
        i++;

        locationSet();

        if(angleDifference_ > angleError_) {
            skibbidyDobDob(i,0.6,0,pfmsConnectorPtr_);  //turning the husky until the the angle difference is less thn the error
            // std::cout << "Look at me turn!" << std::endl;
        } else{
            skibbidyDobDob(i, 0,0.3,pfmsConnectorPtr_); //one all angles align, then it will drive forward to the goal
            // std::cout << "I'm zoomin' " << std::endl;
        }
    }

    return true;
}

bool SkidSteer::setGoal(pfms::geometry_msgs::Point goal){
    //we need to turn type point to type goal
    pfms::geometry_msgs::Goal currentGoal = {goalCount_, goal};
    goal_ = currentGoal;
    pfmsConnectorPtr_->send(currentGoal);
    goalCount_++;

 return true; 
}

bool SkidSteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){

return 0;
}

pfms::PlatformType SkidSteer::getPlatformType(void){
    return type_;
}

double SkidSteer::distanceToGoal(void) {
    
    locationSet();

return absDist_; // which is calculates in locationSet
} 

double SkidSteer::timeToGoal(void){
    locationSet();
    double linearMoveTime = absDist_;
    double fullTime = linearMoveTime + angleDifference_; // adding time required for linear movement to time required for adjusting angle

return fullTime; //return the sum
}


double SkidSteer::distanceTravelled(void){

return 0.0;
}

pfms::nav_msgs::Odometry SkidSteer::getOdometry(void){
    return odo_;
}


void SkidSteer::skibbidyDobDob(unsigned int i, double turn_l_r, double move_f_b, std::shared_ptr<PfmsConnector> pfmsConnectorPtr){
/*.................Grabbed this from snippets code provided............................*/

    // We take the arguments supplied on command line and place them in the uav command message
    // Creating a skidsteer command object with the specified parameters
        pfms::commands::SkidSteer cmd{i,
                                    turn_l_r,
                                    move_f_b}; 

    // Sending the commands from the Skidsteer command type above to the husky bot.
        pfmsConnectorPtr_->send(cmd);

    // We wait for a short time, just to enable remaining of system to respond, recommended to wait
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        bool OK = pfmsConnectorPtr_->read(currentMovement,getPlatformType());

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    