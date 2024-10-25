#include "mission.h"


Mission::Mission(std::vector<ControllerInterface*> controllers)
{
  controllers_ = controllers; // Initialize the controllers vector
  distanceTravelled_ = 0.0; // Initialize the distance travelled
  totalDistance_ = 0.0; // Initialize the total distance
  percentage_ = 0.0; // Initialize the percentage
}

Mission::~Mission(){
  
}

 void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform){



    switch (goalMode_) {
        case mission::Objective::BASIC :
            for (auto& controller : controllers_) {
                if (controller->getPlatformType() == platform) {
                controller->setGoals(goals);
                }
            }
            break;
        
        case mission::Objective::ADVANCED :

            // Generate adjacency list graph
            auto graph = generateGraph(goals);

            std::vector<int> optimizedOrder = brute_force_tsp_for_goals(graph);

            // Rearrange goals based on optimized order
            std::vector<pfms::geometry_msgs::Point> rearrangedGoals;
            for (size_t index : optimizedOrder)
            {
                // std::cout << "LE sorted INDEX: " << index << "\n";
                rearrangedGoals.push_back(goals.at(index));
                platformGoalAssociation_.push_back({platform, index});
            }

            // Set rearranged goals for the platform
            for (auto &controller : controllers_)
            {
                if (controller->getPlatformType() == platform)
                {
                    controller->setGoals(rearrangedGoals);
                }
            }
            break;
    }
 }

  bool Mission::run(){
    for (auto& controller : controllers_) {
       controller->run();
    }

    return true;
  }


std::vector<unsigned int> Mission::status(){
        std::vector<unsigned int> status;


        pfms::PlatformStatus quadStatus = controllers_.at(0)->status();
        pfms::PlatformStatus ackermanStatus = controllers_.at(1)->status();


        if(quadStatus == 0){
                status.push_back(100);
            } else {
                status.push_back(0);
            }

        if(ackermanStatus == 0){
                status.push_back(100);
            } else {
                status.push_back(0);
            }

        return status;
    }

  void Mission::setMissionObjective(mission::Objective objective){
    goalMode_ = objective;
  }

  std::vector<double> Mission::getDistanceTravelled(){
    std::vector<double> distances;
    for(int i = 0; i < controllers_.size(); i++){
        auto distance = controllers_.at(i)->distanceTravelled();
        distances.push_back(distance);
    }

    return distances;
  }

  std::vector<double> Mission::getTimeMoving(){
    std::vector<double> times;
    for(int i = 0; i < controllers_.size(); i++){
        auto time = controllers_.at(i)->timeTravelled();
        times.push_back(time);
    }

    return times;
  }

  std::vector<std::pair<int, int>> Mission::getPlatformGoalAssociation(){
    // std::vector<std::pair<int, int>> associations;

    // for (size_t i = 0; i < controllers_.size(); ++i) {
    //     ControllerInterface* controller = controllers_[i];
    //     associations.emplace_back(i, currentGoalIndex_[i]);
    // }

    // return associations;

    return platformGoalAssociation_;
  }

    void Mission::calculateMissionPercentage() {
    for (const auto& controller : controllers_) {
        distanceTravelled_ += controller->distanceTravelled();
    }

    percentage_ = (distanceTravelled_ / totalDistance_) * 100.0;
    std::cout << "Mission Percentage: " << percentage_ << std::endl;
    std::cout << "Distance Travelled: " << distanceTravelled_ << std::endl;
    std::cout << "Total Distance: " << totalDistance_ << std::endl;
}


  double Mission::calculateTotalDistance(const std::vector<pfms::geometry_msgs::Point>& goals) {
    double totalDistance = 0.0;
    for (size_t i = 0; i < goals.size() - 1; ++i) {
        double dx = goals[i + 1].x - goals[i].x;
        double dy = goals[i + 1].y - goals[i].y;
        totalDistance += std::sqrt(dx * dx + dy * dy);
    }
    return totalDistance;
}




// Generate adjacency list graph for goals
AdjacencyList Mission::generateGraph(const std::vector<pfms::geometry_msgs::Point> &goals)
{
    //For loop to make a vector of the distance of all the points to the origin.
    for (auto controller : controllers_)
    {   
        std::cout<<"Controller size: " << controllers_.size() << "\n";
        for (int i = 0; i < goals.size(); i++)
        {   
            std::cout<<" Inside the calculator loop" << "\n";
            controller->checkOriginToDestination(controller->getOdometry(), goals.at(i), distance_, time_, estimatedGoalPose_);
            std::cout<< "I made it past check origin "<< "\n";

            origin_to_goal_.push_back(distance_);
            std::cout<<"Distance "<<i<< " : " << distance_ <<"\n";


        }
    }



    std::cout<<"lets make a graph"<<"\n";
    AdjacencyList graph(goals.size());
    for (size_t i = 0; i < goals.size(); i++)
    {
        for (size_t j = 0; j < goals.size(); j++)
        {   
            std::cout<<"J for loop"<<"\n";
            if (i != j)
            {
                double distance = calculateDistance(goals[i], goals[j]);
                graph.at(i).push_back(std::make_pair(j, distance));

                std::cout<<"Pushed back distance : " << distance << "\n";
            }
            else
            {
                graph.at(i).push_back(std::make_pair(j, 0));
                std::cout<<"Pushed back distance : 0" << "\n";

            }

        }
    }
    return graph;
}

std::vector<int> Mission::brute_force_tsp_for_goals(const std::vector<std::vector<std::pair<int, double>>> &graph)
{

    std::vector<int> order;
    // Let's list the nodes in a vector, and we can do permutations of
    // it as all of the nodes are connected
    std::vector<int> nodes;
    for (unsigned int i = 0; i < graph.size(); i++)
    {
        nodes.push_back(i);
    }

    // We save the total path as a very large value (default)
    double minDistance = 1e6;

    // This while loop creates all possible permutations of the nodes
    // We can use this while loop to create an order of ID's visited
    // Let's look for te total path to visit nodes in current node order
    //unsigned int i = 0;

    do
    {
        bool OK = true;       // We will use this to abolish search if needed
        unsigned int idx = 1; // Let's start from index 1 to end
        double dist = 0;      // Current distance that we have travelled throug nodes
        while ((idx < nodes.size()) && (OK))
        {
            // We have two nodes
            unsigned int node1 = nodes.at(idx -1);
            unsigned int node2 = nodes.at(idx);
            // We find in adjacency list node 1 connection to node2 and second element
            // in teh pair is the distance between these nodes

            dist += graph.at(node1).at(node2).second; 

            // std::cout << dist << " "; // This printed distance in debug mode
            // we can abolish search if we are already over the min distance
            if (dist > minDistance)
            {
                OK = false;
            }
            idx++; // Otherwise we increment to next node
        }

        // std::cout<<"dist "<< " : " << dist <<"\n";

        dist += origin_to_goal_.at(nodes.at(0));

        // std::cout<<"dist updated "<< " : " << dist <<"\n";
        // std::cout<<"origin to the goal: " << origin_to_goal_.at(nodes.at(0)) <<"\n";

        // std::cout << std::endl;
        if (dist < minDistance)
        {
            minDistance = dist; // Save minimum distance
            order.clear();      // clear the current order of nodes
            order = nodes;      // Save the order of nodes
        }
    } while (std::next_permutation(nodes.begin(), nodes.end()));

    return order;
}

// Helper function to calculate distance between two points
double Mission::calculateDistance(const pfms::geometry_msgs::Point &point1, const pfms::geometry_msgs::Point &point2)
{
    double distX = point2.x - point1.x;
    double distY = point2.y - point1.y;
    return sqrt(pow(distX, 2) + pow(distY, 2));
}


/*--------------------------------------Old Set Goals----------------------------------------*/
            // std::vector<Point> points;
            // for (const auto& goal : goals) {
            //     points.push_back({goal.x, goal.y});
            // }

            // std::vector<std::vector<double>> graph(points.size(), std::vector<double>(points.size(), 0.0));
            // for (int i = 0; i < points.size(); ++i) {
            //     for (int j = 0; j < points.size(); ++j) {
            //         graph[i][j] = distance(points.at(i), points.at(j));
            //     }
            // }

            // // Convert goals to adjacency list format
            // AdjacencyList adjacencyList = exportGraph();
            // // Convert adjacency list to std::vector<std::vector<double>>
            // std::vector<std::vector<double>> doubleGraph = convertToDoubleGraph(adjacencyList);
            // // Find optimal order using brute force TSP
            // std::vector<int> optimalOrder = brute_force_tsp(doubleGraph, 0);

            // for (size_t i = 0; i < controllers_.size(); ++i) {
            //     if (controllers_.at(i)->getPlatformType() == platform) {
            //         std::vector<pfms::geometry_msgs::Point> orderedGoals;
            //         for (size_t j = 0; j < goals.size(); ++j) {
            //             orderedGoals.push_back(goals.at(optimalOrder.at(j)));
            //         }
            //         controllers_.at(i)->setGoals(orderedGoals);
            //         currentGoalIndex_.at(i) = optimalOrder.at(0);
            //     }
            // }
        
/*--------------------------------------Old Advanced Mode-------------------------------------------------*/



// struct Point {
//     double x;
//     double y;
// };

// double distance(const Point& p1, const Point& p2) {
//     double dx = p1.x - p2.x;
//     double dy = p1.y - p2.y;
//     return std::sqrt(dx * dx + dy * dy);
// }

// // Brute force TSP algorithm
// std::vector<int> Mission::brute_force_tsp(const std::vector<std::vector<double>>& graph, int start) {
//     std::vector<int> order; // Initialize the Order Vector
//     std::vector<int> nodes;
//     for (int i = 0; i < graph.size(); ++i) {
//         if (i != start) {
//             nodes.push_back(i);
//         }
//     }

//     double minDistance = std::numeric_limits<double>::max();
//     do {
//         double dist = 0;
//         int prevNode = start;
//         for (int i = 0; i < nodes.size(); ++i) {
//             int currentNode = nodes.at(i);
//             dist += graph.at(prevNode).at(currentNode);
//             prevNode = currentNode;
//         }
//         dist += graph[prevNode][start];
//         if (dist < minDistance) {
//             minDistance = dist;
//             order = nodes;
//         }
//     } while (std::next_permutation(nodes.begin(), nodes.end()));

//     order.insert(order.begin(), start);
//     return order;
// }

// AdjacencyList Mission::exportGraph() {
//     AdjacencyList graph;

//     for (unsigned int i = 0; i < goals_.size(); i++) {
//         std::vector<std::pair<int, double>> edges;
//         for (unsigned int j = 0; j < goals_.size(); j++) {
//             if (i != j) {
//                 // Calculate the absolute distance using the Euclidean formula
//                 double distance = std::sqrt(std::pow(goals_.at(j).x - goals_.at(i).x, 2) + std::pow(goals_.at(j).y - goals_.at(i).y, 2));
//                 // Add edge to the created edges vector
//                 edges.push_back({static_cast<int>(j), distance});
//             }
//         }
//         // Add edges for goal onto the graph
//         graph.push_back(edges);
//     }

//     return graph;
// }

// // Convert AdjacencyList to std::vector<std::vector<double>>
// std::vector<std::vector<double>> Mission::convertToDoubleGraph(const AdjacencyList& graph) {
//     std::vector<std::vector<double>> doubleGraph(graph.size());
//     for (size_t i = 0; i < graph.size(); ++i) {
//         for (const auto& edge : graph.at(i)) {
//             doubleGraph.at(i).push_back(edge.second);
//         }
//     }
//     return doubleGraph;
// }