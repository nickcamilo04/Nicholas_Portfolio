#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"
#include <iostream>
#include <stdlib.h>
#include <chrono>

class Ackerman: public Controller
{
public:
  //Default constructor should set all sensor attributes to a default value
  Ackerman();

  /**
   * @brief Destructor for the Ackerman class
   */
  ~Ackerman();

  void run(void);
  pfms::PlatformStatus status(void);
  bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose);


  pfms::PlatformType getPlatformType(void);
  double distanceToGoal(void);
  double timeToGoal(void);
  pfms::nav_msgs::Odometry getOdometry(void);
  std::vector<pfms::geometry_msgs::Point> getObstacles(void);

  private:
  
  /**
 * @brief Executes the main loop for the Ackerman platform
 * @return True if execution was successful, false otherwise
 */
  bool audiZooms();

  pfms::PlatformType platformType_; /**< Platform type */
  pfms::nav_msgs::Odometry origin_; /**< Origin odometry */
  pfms::nav_msgs::Odometry currentMovement; /**< Current movement */
  pfms::nav_msgs::Odometry estimatedGoalPose_; /**< Estimated goal pose */
  pfms::geometry_msgs::Point initial_point_; /**< Initial point */
  pfms::geometry_msgs::Point current_point_; /**< Current point */
  pfms::geometry_msgs::Goal goal_; /**< Goal */
  std::vector<pfms::geometry_msgs::Point> goalVec_; /**< Vector of goals */
  pfms::PlatformStatus status_; /**< Status */
  uint8_t goalUpdate_; /**< Goal update */
  double distance_; /**< Distance */
  double absVelocity_; /**< Absolute velocity */
  pfms::geometry_msgs::Vector3 initialVelocity_; /**< Initial velocity */
  int repeats_; /**< Repeats */
  double brake_; /**< Brake */
  double steering_; /**< Steering */
  double throttle_; /**< Throttle */
  pfms::nav_msgs::Odometry odo_; /**< Odometry */
  std::vector<std::thread> threads_; /**< Threads */
  std::atomic<bool> running_; /**< Running flag */
  std::atomic<bool> ready_; /**< Ready flag */
  std::mutex mtx_; /**< Mutex */
  std::condition_variable cv_; /**< Condition variable */
  std::condition_variable cvStart_; /**< Condition variable for starting */
  std::mutex mtxStart_; /**< Mutex for starting */

  Audi audi;

};

#endif // ACKERMAN_H
