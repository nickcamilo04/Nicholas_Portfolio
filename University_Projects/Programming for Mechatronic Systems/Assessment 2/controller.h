#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>
#include <thread>
#include <mutex>
// #include "pfmsconnector.h"

class Controller: public ControllerInterface
{
  public:
  //Default constructors should set all attributes to a default value
  Controller();

  //See controllerinterface.h for more information

  virtual void run(void)=0;
  virtual pfms::PlatformStatus status(void)=0;
  virtual bool setGoals(std::vector<pfms::geometry_msgs::Point> goals)=0;
  virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose)=0;


  virtual pfms::PlatformType getPlatformType(void)=0;
  virtual double distanceToGoal(void)=0;
  virtual double timeToGoal(void)=0;
  // virtual bool setTolerance(double tolerance)=0;
  // virtual double distanceTravelled(void)=0;
  // virtual double timeTravelled(void)=0;
  virtual pfms::nav_msgs::Odometry getOdometry(void)=0;
  virtual std::vector<pfms::geometry_msgs::Point> getObstacles(void)=0;

  bool setTolerance(double tolerance);
  double timeTravelled();
  double distanceTravelled();

  protected:
   std::shared_ptr<PfmsConnector> pfmsConnectorPtr_;
   double toleranceError_;
   double time_;  //We will have a time variable to store how much time has gone by 
   pfms::geometry_msgs::Goal goal_;
   pfms::nav_msgs::Odometry currentMovement_;
};

#endif // CONTROLLER_H
