#include "pfms_types.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "ackerman.h"
#include "quadcopter.h"
#include "mission.h"
#include "logger.h"

//using namespace std;
using namespace pfms::nav_msgs;

int main(int argc, char *argv[]) {

        //Default location of files containing sequence of points and objective
   std::string ackerman_filename = "../data/ACKERMAN.TXT";
   std::string quadcopter_filename = "../data/QUADCOPTER.TXT";
   mission::Objective objective =  mission::Objective::BASIC;

   // Will default to the settings unless specified with ovvride as arguments
   // Example ./assignment2 --advanced $HOME/data/ACKERMAN.TXT $HOME/data/QUADCOPTER.TXT
   // This woudl assume your files are in home directory in data subfolder
   // You do need the -- (two "-") to enable ADVANCED MODE
   if(argc<4){
        std::cout << "To override Run with 3 arguments: " << std::endl;
        std::cout << argv[0] << " MODE (--advanced) location_of_ACKERMAN.txt location_of_QUADCOPTER.txt" << std::endl;
    }
   else{
       if(strcmp(argv[1],"-advanced")){
           std::cout << "Advanced Mode Activated" << std::endl;
           objective =  mission::Objective::ADVANCED;
       }
       if(strcmp(argv[1],"-super")){
           std::cout << "Advanced Mode Activated" << std::endl;
           objective =  mission::Objective::SUPER;
       }       
       ackerman_filename = argv[2];
       quadcopter_filename = argv[3];

   }

   std::vector<pfms::geometry_msgs::Point> ackermanPoints;
   std::vector<pfms::geometry_msgs::Point> quadcopterPoints;

   //If the files can not be opened we will terminate
    if(!logger::loadPoints(ackerman_filename,ackermanPoints)){
        std::cout << "Could not load points from file:" << ackerman_filename << std::endl;
        return 0;
    }

    if(!logger::loadPoints(quadcopter_filename,quadcopterPoints)){
        std::cout << "Could not load points from file:" << quadcopter_filename << std::endl;
        return 0;
    }

    std::cout << "Size of Ackerman goals:" << ackermanPoints.size() << std::endl;
    std::cout << "Size of Quadcopter goals:" << quadcopterPoints.size() << std::endl;
    
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);

    /*.....................My test implementations........................*/
    // controllers.at(1)->run();
    
    Mission mission(controllers);
    mission.setMissionObjective(objective);
    mission.setGoals(quadcopterPoints,pfms::QUADCOPTER);
    mission.setGoals(ackermanPoints,pfms::ACKERMAN);

    mission.run();

 // indicating IDLE, we know it has been reached
    auto start_time = std::chrono::system_clock::now();

    // We start now
    bool OK =false; // This will indicate if mission is completed

    while (!OK){

        std::vector<unsigned int> progress = mission.status();

        if((progress.front() == 100) && (progress.back() == 100)){ // we check progess, at 100 mission is acomplished
            //mission accomplished
            OK=true;
        }
        else{
            //Let's slow down this loop to 200ms (5Hz)
            std::cout << "progress ..." << progress.front() << "% " <<  progress.back() << "% " << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

    }

    for (auto controller : controllers){
        delete controller;
    }

}
