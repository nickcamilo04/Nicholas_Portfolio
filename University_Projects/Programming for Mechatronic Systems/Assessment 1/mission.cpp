#include "mission.h"

Mission::Mission(std::vector<ControllerInterface*> controllers) :
    controllers_(controllers)
    
{}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals){

    goals_.push_back(goals.at(0)); //adding first two goals elements to the vector goals_
    goals_.push_back(goals.at(1));

}

bool Mission::runMission(){   

    for (auto controller : controllers_) {   //start loop over each controller in Controllers_ vector
        controller->setGoal(goals_.at(0));   //calls setGoal and pass the first parameter
        controller->reachGoal();             //triggersthe controller to start moving towards its set goal
        controller->setGoal(goals_.at(1));
        controller->reachGoal();
    }
    
    return true;
}

void Mission::setMissionObjective(mission::Objective objective){

}

std::vector<double> Mission::getDistanceTravelled(){
    std::vector<double> distances;

    return distances;
}

std::vector<double> Mission::getTimeMoving(){
    std::vector<double> times;

    return times;
}

std::vector<unsigned int> Mission::getPlatformGoalAssociation(){
    std::vector<unsigned int> associations;

        // Assuming controllers is a vector of controller objects
    for (const auto& goal : goals_) {
        
        // Store the platform number in the vector
        associations.push_back(0);
        // associations.push_back(1);
    }

    return associations;
}