#include "ackerman.h"

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#define DEBUG 1
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_DEBUG RCUTILS_LOG_DEBUG

using std::cout;
using std::endl;

using std::placeholders::_1; // Place holder

using namespace std::chrono_literals;

// Constructor
Ackerman::Ackerman() : throttle_(0.15),
                       brake_(0),
                       steering_(0),
                       distance_(0),
                       time_(0),
                       goalsReached_(false),
                       goalSet_(false),
                       dist_travelled_(0),
                       time_travelled_(0),
                       cmd_seq_(0),
                       goalCounter_(0),
                       goalCounter2_(0),
                       running_(false),
                       tolerance_(0.5)
{
    // type_ = (pfms::PlatformType::ACKERMAN),                                                                // Platform in use is Ackerman
    // origin_ = {0, {0, 0, 0}, 0},                                                                           // Initialising origin values, default all 0's
    // time_ = 0;                                                                                             // Initialise time variable
    // brake_ = 0;                                                                                            // Initialise brake variable
    // throttle_ = 0;                                                                                         // Initialise throttle variable
    // steering_ = 0;                                                                                         // Initialise steering variable
    // goalsReached_ = false;                                                                                 // Initialise running to false so that the program runs until this is changed
    // running_ = false;                                                                                      // Aids with exiting the code whith threading.
    VecThreads_.push_back(std::thread(&Ackerman::goAudi, this));                                           // Initialising a thread that will run the goAudi function and pushing it back to the threads vector
    // status_ = pfms::PlatformStatus::RUNNING;                                                               // initialising the status as running so that the program operates correctly.

    // Will hold on to the last 3 messages sent to ensure they are sent through?
    // Publisher to the orange brake, steering, and throttle command topics, which make it move
    brake_pub_ = this->create_publisher<std_msgs::msg::Float64>("/orange/brake_cmd", 3);
    steering_pub_ = this->create_publisher<std_msgs::msg::Float64>("/orange/steering_cmd", 3); // what about steering_state
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float64>("/orange/throttle_cmd", 3);

    // Subscribing to the odometry readings of the actual audi in the simulation, used in a callback to update the position_ class variable constantly
    // Will use that for checking tolerancing and so on.
    // The placeholder ensures that the message of the /orange/odom topic is actually passed in as the arguement of odomCallback
    // sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/orange/odom", 1000, std::bind(&Ackerman::odomCallback,this,std::placeholders::_1));

    sub_Odom_ = this->create_subscription<nav_msgs::msg::Odometry>
    ("/orange/odom", 1000, std::bind(&Ackerman::odoCallback, this, std::placeholders::_1));

    sub_Goal_ = this->create_subscription<geometry_msgs::msg::PoseArray>
    ("/orange/goals", 1000, std::bind(&Ackerman::setGoal, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&Ackerman::run, this));
}

// void Ackerman::odomCallback(const nav_msgs::msg::Odometry& msg){
//     std::unique_lock<std::mutex> lock(odom_mutex_);
//     position_ = msg.pose.pose;
// }

// Call positon_ from controller so dont need a odomCallback anymore

void Ackerman::sendCmdAck(double brake, double steering, double throttle)
{
    // Creating the data type that will be sent to ros
    std_msgs::msg::Float64 pubBrake;
    std_msgs::msg::Float64 pubSteering;
    std_msgs::msg::Float64 pubThrottle;
    // Setting the data of the above variables to the input arguement of this function
    pubBrake.data = brake;
    pubSteering.data = steering;
    pubThrottle.data = throttle;
    // Publishing to the correct topics so that the car moves in the sim
    // Need to use audi library I think
    brake_pub_->publish(pubBrake);
    steering_pub_->publish(pubSteering);
    throttle_pub_->publish(pubThrottle);
}

// Destructor
Ackerman::~Ackerman()
{
    // Clean up the pfms connector
    running_ = false;                                                                                      // Signal for the threads to stop

    // Join all threads created by Ackerman, wait for them to complete
    for (auto &thread : VecThreads_)
    {
        thread.join();
    }
}



// Run the platform
void Ackerman::run()
{
    if(goalSet_ && !running_){
        RCLCPP_INFO_STREAM(this->get_logger(), "INSIDE RUN");
        running_ = true;                                                                                        // Set the running flag to true, used for running and exiting the goAudi while loop
        std::unique_lock<std::mutex> lck(mutexStart_);                                                          // Aquire a lock on the mutexStart_ mutex
        mutexStart_.unlock();                                                                                   // Unlock the mutexStart_ mutex
        cvStart_.notify_all();                                                                                  // Notify all threads waiting on cvStart_ that the condition has been met        
    }
    // RCLCPP_INFO_STREAM(this->get_logger(), "LEFT RUN");
                                                                

}


void Ackerman::setGoal(const geometry_msgs::msg::PoseArray msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Controller recived");

    if (goalSet_ == false)
    {
        geometry_msgs::msg::PoseArray goalArray = msg;
        for (auto goalr : goalArray.poses)
        {
            pfms::geometry_msgs::Point goal;
            goal.x = goalr.position.x;
            goal.y = goalr.position.y;
            goal.z = goalr.position.z;
            goals_.push_back(goal);
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "goals_ size: " << goals_.size());
        RCLCPP_INFO_STREAM(this->get_logger(), "Controller set goal");
        goalSet_ = true;
    }
}

bool Ackerman::goalReached()
{
    geometry_msgs::msg::Pose pose = getOdometry(); // This will update internal copy of odometry, as well as return value if needed.
    // Calculate absolute travel angle required to reach goal

    double dx = goals_.at(goalCounter_).x - pose.position.x;
    double dy = goals_.at(goalCounter_).y - pose.position.y;

    return (pow(pow(dx, 2) + pow(dy, 2), 0.5) < tolerance_);
}

void Ackerman::odoCallback(const nav_msgs::msg::Odometry msg)
{
    std::unique_lock<std::mutex> lck(mutex2_);
    position_ = msg.pose.pose;
    
    odomAudi_.position.x = position_.position.x;
    odomAudi_.position.y = position_.position.y;
    odomAudi_.position.z = position_.position.z;

    double quatToyaw = tf2::getYaw(position_.orientation);
    odomAudi_.yaw = quatToyaw;

    // RCLCPP_INFO_STREAM(this->get_logger(), "Odom x: " << odomAudi_.position.x);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Odom y: " << odomAudi_.position.y);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Odom z: " << odomAudi_.position.z);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Odom YAW: " << odomAudi_.yaw);

}

// Get the distance to the goal
double Ackerman::distanceToGoal()
{
    checkOriginToDestination(odomAudi_, goals_.at(goalCounter_), distance_, time_, estimatedGoalPose_); // Passes distance_ in by reference which updates the distance to the goal Audi needs to go to
    //RCLCPP_INFO_STREAM(this->get_logger(), "---Distance to goal: " << distance_);
    return distance_;                                                                                   // Returns distance_ which represetns the distance to the goal position
}

// Checking if the audi has reached the goal
bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal, double &distance, double &time, pfms::nav_msgs::Odometry &estimatedGoalPose)
{
    return audi_.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose); // By passing the values in by reference they are updated using the audi library
}




// Function drives the ackerman platform to the goal
bool Ackerman::goAudi()
{
        std::unique_lock<std::mutex> lck(mutexStart_);
        cvStart_.wait(lck, [&]() { return running_ == true; });

        while (running_ == true)
        {
            audi_.computeSteering(odomAudi_, goals_.at(goalCounter_), steering_, distance_); // Constantly reading the steering angle needed to get to the next goal (updating as we go)
            sendCmdAck(brake_, steering_, throttle_);

            unsigned long iterator = 0;                 // Initialising iterator, used to display number of steps loop has taken
            double brakeConstant = 2500;                // Initialising a brake constant which is used when braking
            double realisticTolerance = 3 * tolerance_; // Due to inertia, the car cannot stop promptly and requires a more realistic tolerance (3 times the tolerance value) when reaching goal
            int goalio = goals_.size() - 1;             // Value to help keep track of which goal the audi is upto and when to break out of the loop

            RCLCPP_INFO_STREAM(this->get_logger(), "AUDI RUN INIT");
            RCLCPP_INFO_STREAM(this->get_logger(), "Goal X: " << goals_.at(goalCounter_).x);
            RCLCPP_INFO_STREAM(this->get_logger(), "Goal Y: " << goals_.at(goalCounter_).y);
            RCLCPP_INFO_STREAM(this->get_logger(), "Goal Z: " << goals_.at(goalCounter_).z);

            // Drive to the goals while the goalsReached_ variable is false
            while (goalsReached_ == false) // This value will be set to true when all the goals are reached, successfully braking out of the loop
            {

                // geometry_msgs::msg::Pose origin = getOdometry(); // This will update internal copy of odometry, as well as return value if needed.
                // Mapping the origin to pfms type

                // odomAudi_.position.x = origin.position.x;
                // odomAudi_.position.y = origin.position.y;
                // // odomAudi_.linear.z = origin.position.z;

                // double siny_cosp = 2.0 * (origin.orientation.w * origin.orientation.z + origin.orientation.x * origin.orientation.y);
                // double cosy_cosp = 1.0 - 2.0 * (origin.orientation.y * origin.orientation.y + origin.orientation.z * origin.orientation.z);
                // odomAudi_.yaw =  std::atan2(siny_cosp, cosy_cosp)

                audi_.computeSteering(odomAudi_, goals_.at(goalCounter_), steering_, distance_); // Constantly reading the steering angle needed to get to the next goal (updating as we go)

                // If the distance to goal is less than or equal to 3 times the tolerance which = 1.5m will begin braking
                if (distanceToGoal() <= realisticTolerance)
                {
                    brake_ = brakeConstant / distanceToGoal(); // Will brake dynamically starting at 1.5m away from the goal, reaching max braking when distance to goal is just before 0m
                }
                else
                {
                    brake_ = 0; // Brake set to 0 if the goal is now passed, or distance to goal is greater than adjusted tolerance
                }

                // Check if the distanceToGoal is within the tolerance if it is and the goalCounter_ value is less than goalio which is the goal size then increment to next goal and begin moving there
                if (distanceToGoal() <= tolerance_ && goalCounter_ < goalio)
                {
                    goalCounter_++; // Increment the goal counter for audi to begin moving to the next goal
                    RCLCPP_INFO_STREAM(this->get_logger(), "Goal Counter: " << goalCounter_);
                }

                // Check if the tolerance limit is met and if the goalCounter_ is at the final goal
                if (distanceToGoal() <= tolerance_ && goalCounter_ == goalio)
                {
                    goalsReached_ = true; // Flag to say all goals have been reached and break out of this while loop
                }

                sendCmdAck(brake_, steering_, throttle_);

            }
            // Checking if running is set to false
            if (!running_)
            {
                break;                                                                                          // If running is false break out of the while(running == true) while loop
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));                                         // Thread sleep for 10 seconds to ensure the previous commands have been sent through
            break;  
        }
    // break out of the while loop
    return true;
}