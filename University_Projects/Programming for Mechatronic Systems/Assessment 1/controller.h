#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pfmsconnector.h"

class Controller: public ControllerInterface
{
public:
  //Default constructors should set all attributes to a default value
  Controller();
  ~Controller();

  //See controllerinterface.h for more information
  virtual bool reachGoal(void) = 0;
  virtual bool setGoal(pfms::geometry_msgs::Point goal) = 0;
  virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose) = 0;
  virtual pfms::PlatformType getPlatformType(void) = 0;
  virtual double distanceToGoal(void) = 0;
  virtual double timeToGoal(void) = 0;
  virtual double distanceTravelled(void) = 0;
  // virtual double timeInMotion(void) = 0;
  virtual pfms::nav_msgs::Odometry getOdometry(void) = 0;

  bool setTolerance(double tolerance);
  virtual double timeInMotion();
protected:
  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_;
  double toleranceError_;
  double time_;  //We will have a time variable to store how much time has gone by 
};

#endif // CONTROLLER_H
