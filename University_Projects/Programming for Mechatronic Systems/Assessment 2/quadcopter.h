#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"
#include <iostream>
#include <stdlib.h>
#include <chrono>

class Quadcopter: public Controller
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Quadcopter();

  /**
 * @brief Destructor for the Quadcopter class
 */
  ~Quadcopter();
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
   * @brief Executes the main loop for the quadcopter platform
   * @return True if execution was successful, false otherwise
   */
  bool quadTakeoff(); 


  pfms::PlatformType platformType_; /**< Platform type */
  pfms::nav_msgs::Odometry origin_; /**< Origin odometry */
  pfms::nav_msgs::Odometry currentMovement; /**< Current movement */
  pfms::nav_msgs::Odometry estimatedGoalPose_; /**< Estimated goal pose */
  pfms::geometry_msgs::Point initial_point_; /**< Initial point */
  pfms::geometry_msgs::Point current_point_; /**< Current point */
  pfms::geometry_msgs::Goal goal_; /**< Goal */
  pfms::PlatformStatus status_; /**< Status */

  uint8_t goalUpdate_; /**< Goal update */
  std::vector<pfms::geometry_msgs::Point> goalVec_; /**< Vector of goals */
  double actualTolerance_; /**< Tolerance */
  bool goalsReached_ = false; /**< Goals reached flag */

  unsigned long seq; /**< Sequence number of command */
  double turn_l_r; /**< Angular speed of turn */
  double move_l_r; /**< Speed of left/right motion */
  double move_u_d; /**< Speed of up/down motion */
  double move_f_b; /**< Speed of forward/backward motion */
  double speed_; /**< Speed */

  std::vector<std::thread> threads_; /**< Threads */
  std::atomic<bool> running_; /**< Running flag */
  std::atomic<bool> ready_; /**< Ready flag */
  std::mutex mtx_; /**< Mutex */

  std::condition_variable cv_; /**< Condition variable */
  std::condition_variable cvStart_; /**< Condition variable for starting */
  std::mutex mtxStart_; /**< Mutex for starting */
};

#endif // QUADCOPTER_H
