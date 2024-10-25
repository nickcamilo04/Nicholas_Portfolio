#include "ackerman.h"
#include "skidsteer.h"
#include "mission.h"
#include <vector>
#include "pfms_types.h"
#include <iostream>

using std::vector;
using std::cout;
using std::endl;

int main(int argc, char *argv[]) {

    double x1=2;
    double y1=5;
    double x2=4;
    double y2=-5;

    if(argc !=3){
         cout << " Not arguments given on command line." << endl;
         cout << " usage: " << argv[0] << "<x> <y>" << endl;
         cout << " defaulting to: " << x1 << " " << y1  << " " << x2 << " " << y2 << endl;
    }
    else{
         x1 = atof(argv[1]);
         y1 = atof(argv[2]);
         x2 = atof(argv[1]);
         y2 = atof(argv[2]);

         cout << " Goals : " << x1 << " " << y1  << " " << x2 << " " << y2 << endl;

    }
    
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new SkidSteer());


    //Setting tolerance to reach goals
    controllers.at(0)->setTolerance(0.5);
    controllers.at(1)->setTolerance(0.5); 


    //Goals
    pfms::geometry_msgs::Point goal0{x1,y1};
    pfms::geometry_msgs::Point goal1{x2,y2};

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back(goal0);
    goals.push_back(goal1);


    //////////////////////////////////////////////////////////////////////////////////
    // Let's now check missions
    Mission mission(controllers);
    mission.setGoals(goals);

    mission.setMissionObjective(mission::Objective::DISTANCE);

    std::vector<unsigned int> assignment =  mission.getPlatformGoalAssociation();

    for(unsigned int i=0;i<assignment.size();i++){
        std::cout << i << " : " << assignment.at(i) << std::endl;
    }

    bool OK = mission.runMission();

    if(OK){
        std::cout << "Controllers did reach goals" << std::endl;
    }
    else {
        std::cout << "Controller CAN NOT reach goals" << std::endl;
    }

    //The and skidsteer should be within 0.5m of goal position when completing motion.


    return 0;
}
