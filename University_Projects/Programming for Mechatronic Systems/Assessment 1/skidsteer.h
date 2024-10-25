#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

class SkidSteer: public Controller
{
public:
  //Default constructor should set all attributes to a default value
  SkidSteer();

  bool reachGoal();
  bool setGoal(pfms::geometry_msgs::Point goal);
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose);
  pfms::PlatformType getPlatformType(void);
  double distanceToGoal(void);
  double timeToGoal(void);
  double distanceTravelled(void);

  pfms::nav_msgs::Odometry getOdometry(void);


 /**
 * @brief Performs the skibbidyDobDob operation with specified parameters.
 *
 * This function executes the drive operation using the provided parameters
 * to control the movement and turning. It communicates with the PfmsConnector object
 * through the provided shared pointer.
 *
 * @param i The unsigned integer parameter for the skibbidyDobDob operation.
 * @param turn_l_r The double parameter representing the turning direction (left/right).
 * @param move_f_b The double parameter representing the movement direction (forward/backward).
 * @param pfmsConnectorPtr A shared pointer to the PfmsConnector object for communication.
 */
  void skibbidyDobDob(unsigned int i, double turn_l_r, double move_f_b, std::shared_ptr<PfmsConnector> pfmsConnectorPtr);

  /**
  * @brief Performs geographical positioning calculations to determine an acurate location of the Husky
  * 
  * @return the geographical location of the husky
  */
  void locationSet();
 
private:
  pfms::PlatformType platformType_;
  pfms::nav_msgs::Odometry origin_;
  pfms::nav_msgs::Odometry currentMovement;
  pfms::nav_msgs::Odometry estimatedGoalPose_;
  pfms::geometry_msgs::Point initial_point_;
  pfms::geometry_msgs::Point current_point_; 
  pfms::geometry_msgs::Point goal_point_; 
  pfms::geometry_msgs::Goal goal_;

  double distance_;

  pfms::geometry_msgs::Vector3 initialVelocity_;
  

  pfms::nav_msgs::Odometry odo_;
  pfms::PlatformType type_;


  unsigned long goalCount_;
  double absDist_;
  double angleDifference_;
  double angleError_;
  double XDist_;
  double YDist_;
  double goalAngle_;
  double currentAngle_;
  double absVelocity_;

};

#endif // SKIDSTEER_H
