#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"
#include "pfmsconnector.h"
#include <thread>
#include <mutex>
#include <algorithm>
#include <random>
#include <utility>
#include <cmath> 

typedef std::vector<std::vector<std::pair<int, double>>> AdjacencyList;

class Mission: public MissionInterface
{
public:
    /**
    The Default constructor
    @sa ControllerInterface and @sa MissionInterface for more information
    */
  Mission(std::vector<ControllerInterface*> controllers);
  
  /**
   * @brief Destructor for the Mission class.
   */
  ~Mission();

  void setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform);
  bool run();
  std::vector<unsigned int> status(void);
  void setMissionObjective(mission::Objective objective);
  std::vector<double> getDistanceTravelled();
  std::vector<double> getTimeMoving();
  std::vector<std::pair<int, int>> getPlatformGoalAssociation();






private:

//TSP Workings

/**
 * @brief Solves the Traveling Salesman Problem (TSP) for a given set of goals using a brute force approach.
 * 
 * This function computes the shortest tour that visits each goal exactly once and returns the sequence of goals in the optimal order.
 * 
 * @param graph A weighted adjacency list representing the graph. Each element of the outer vector corresponds to a vertex in the graph, 
 *              containing a vector of pairs representing the neighboring vertices and their respective edge weights.
 * @return A vector containing the sequence of goals in the optimal order according to the TSP solution.
 */
std::vector<int> brute_force_tsp_for_goals(const std::vector<std::vector<std::pair<int, double>>> &graph);

/**
 * @brief Calculates the Euclidean distance between two points in 2D space.
 * 
 * This function computes the Euclidean distance between two points represented by pfms::geometry_msgs::Point objects.
 * 
 * @param point1 The first point.
 * @param point2 The second point.
 * @return The Euclidean distance between point1 and point2.
 */
double calculateDistance(const pfms::geometry_msgs::Point &point1, const pfms::geometry_msgs::Point &point2);

/**
 * @brief Generates an adjacency list representation of a graph from a set of goals.
 * 
 * This function constructs a graph representation suitable for solving the Traveling Salesman Problem (TSP) 
 * based on a set of goals provided as pfms::geometry_msgs::Point objects.
 * 
 * @param goals A vector containing the coordinates of the goals in the graph.
 * @return An adjacency list representing the graph. Each element of the outer vector corresponds to a vertex in the graph, 
 *         containing a vector of pairs representing the neighboring vertices and their respective edge weights.
 */
AdjacencyList generateGraph(const std::vector<pfms::geometry_msgs::Point> &goals);


  std::vector<ControllerInterface*> controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  //std::vector<pfms::geometry_msgs::Point*> goals_; //!< A private copy of goals
  std::vector<pfms::geometry_msgs::Point> goals_; //!< A private copy of goals

  std::vector<std::vector<int>> platformOrder_; //order of goals for each platform
  

  /**
   * @brief Calculates the percentage completion of the mission.
   */
  void calculateMissionPercentage();

  /**
   * @brief Calculates the total distance to be covered in the mission.
   * 
   * @param goals The vector of goals defining the mission path.
   * @return The total distance to be covered.
   */
  double calculateTotalDistance(const std::vector<pfms::geometry_msgs::Point>& goals);
  
  double toleranceError_; //!< Tolerance error value.
  double totalDistance_; //!< Total distance to be covered.
  double completedDistance_; //!< Completed distance.
  bool missionCompleted_; //!< Flag indicating if the mission is completed.
  std::vector<int> currentGoalIndex_; //!< Current goal index for each platform.
  double distanceTravelled_; //!< Distance travelled during the mission.
  double percentage_; //!< Percentage completion of the mission.
  int goalMode_ = 2; //!< Mission objective mode.
  int statusMode_ = 2;

  pfms::nav_msgs::Odometry estimatedGoalPose_; /**< Estimated goal pose */



  //TSP Workings
  std::vector<std::pair<int, int>> platformGoalAssociation_; // Association of platform index with goal index
  std::vector<double> origin_to_goal_; /**< Vector containing the distances from the origin to each goal*/

  double distance_; /**< Distance */
  double time_; /**< Time */

};

#endif // RANGERFUSION_H
