/***************************************************************************************************************************
* outdoorwifitest_ground.cpp
*
* Author: Longhao Qian
*
* Update Time: 2020.10.03
*
* Introduction:  An outdoor wifi communincation testing tool. It sends out a fake drone state and a fake
* auxiliary state at 50Hz to the ground station to test the wifi quality
* computer
* This node should run on the ground station
***************************************************************************************************************************/
#include <iostream>
#include <string>
#include <functional>

#include <ros/ros.h>
#include <px4_command/AuxiliaryState.h>
#include <px4_command/FleetStatus.h>
#include <px4_command_utils.h>


void SubAuxiliaryState(const px4_command::AuxiliaryState::ConstPtr& msg, bool* IsReceived, px4_command::AuxiliaryState& state){
    *IsReceived = true;
    state = *msg;
}

void SubFleetStatus(const px4_command::FleetStatus::ConstPtr& msg, bool* IsReceived, px4_command::FleetStatus& status){
    *IsReceived = true;
    status = *msg;
}

void DispConnectionQuality(int& counter, std::string& name){
    std::cout << name << " quality is: ";
    if(counter<2){
        std::cout<< "Excellent. " << "\n";
    }else if (counter <5){
        std::cout<< "Good. " << "\n";
    }else if (counter <8) {
        std::cout<< "Poor. " << "\n";
    }else{
        std::cout<< "Unstable. " << "\n";
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "outdoorwifitest_ground");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);

    float CurrentTime = 0.0;
    ros::Time begin_time = ros::Time::now();

    int tickfordisplay = 0;
    int ticknow = 0;
    bool displaytimestamp = false;
    int timedispstep = 1;
    bool IsAuxiliaryReceived = false;
    bool IsFleetStatusReceived = false;

    px4_command::AuxiliaryState FakeAuxstate;
    px4_command::FleetStatus    FaKeFleetStatus;

    namespace arg = std::placeholders;

    ros::Subscriber  subAuxiliaryState =  nh.subscribe<px4_command::AuxiliaryState>("/test/px4_command/auxiliarystate", 
                                                                            100, 
                                                                            std::bind(&SubAuxiliaryState, arg::_1, 
                                                                            &IsAuxiliaryReceived, FakeAuxstate));
    ros::Subscriber  subFleetStatus = nh.subscribe<px4_command::FleetStatus>("/test/px4_command/fleetstatus",
                                                            100,
                                                            std::bind(&SubFleetStatus,arg::_1,
                                                            &IsFleetStatusReceived , FaKeFleetStatus ));
                                                            
    int AuxiliaryStateLossCounter = 10;
    int FleetStatusLossCounter = 10;
    std::string AuxName = "auxiliary state";
    std::string FleetName = "fleet status ";
    while(ros::ok()) {

        if(IsAuxiliaryReceived){
            AuxiliaryStateLossCounter = 0; //reset the 
            IsAuxiliaryReceived = false;
        }else{
            if (AuxiliaryStateLossCounter<20){
                AuxiliaryStateLossCounter++;                
            }
        }
        if(IsFleetStatusReceived){
            FleetStatusLossCounter = 0; //reset the 
            IsFleetStatusReceived = false;
        }else{
            if (FleetStatusLossCounter<20){
                FleetStatusLossCounter++;                
            }
        }
        CurrentTime = px4_command_utils::get_time_in_sec(begin_time);
        ticknow = int(floor(CurrentTime)); // record the tick
        if((ticknow % timedispstep == 0)) { // ticks now is a multiple of 10
            if(tickfordisplay == ticknow){ 
                // if the display tick is equal to the current tick, display time and set the advance the tickfordisplay by 1
                std::cout<< "--------------------------- \n";
                std::cout<< " Time: " << CurrentTime << " [s]. \n";
                DispConnectionQuality(AuxiliaryStateLossCounter, AuxName);
                DispConnectionQuality(FleetStatusLossCounter, FleetName);
                tickfordisplay += timedispstep;
            }
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;


}