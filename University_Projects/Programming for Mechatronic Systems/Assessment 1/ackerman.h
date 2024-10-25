#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

class Ackerman: public Controller
{
public:
  //Default constructor should set all attributes to a default value
  Ackerman();


  bool reachGoal(void);
  bool setGoal(pfms::geometry_msgs::Point goal);
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose);
  pfms::PlatformType getPlatformType(void);
  virtual double distanceToGoal(void);
  double timeToGoal(void);
  double distanceTravelled(void);
  pfms::nav_msgs::Odometry getOdometry(void);
  

  /**
  * @brief The drive function for the Audi. Uses tollerances and distances to compute breaking and throttle points
  * 
  * @return drive function using Audi Library
  */
  bool zoomZooms();


private:
  pfms::PlatformType platformType_;
  pfms::nav_msgs::Odometry origin_;
  pfms::nav_msgs::Odometry currentMovement;
  pfms::nav_msgs::Odometry estimatedGoalPose_;
  pfms::geometry_msgs::Point initial_point_;
  pfms::geometry_msgs::Point current_point_; 
  pfms::geometry_msgs::Goal goal_;

  double distance_;

  pfms::geometry_msgs::Vector3 initialVelocity_;

  int repeats_; 
  double brake_;
  double steering_;
  double throttle_;

  pfms::nav_msgs::Odometry odo_;

};

#endif // ACKERMAN_H
