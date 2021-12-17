/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "aliengo_sdk/aliengo_sdk.hpp"
#include "convert.h"

// using namespace UNITREE_LEGGED_SDK;

class CmdClass {
    public:
        CmdClass();

        void CmdCallback(unitree_legged_msgs::HighCmd);
        
        unitree_legged_msgs::HighCmd Read();

        unitree_legged_msgs::HighCmd lastCmd;
        boost::mutex mutex;
};

/** Class constructor. */
CmdClass::CmdClass()
{
    lastCmd.forwardSpeed = 0.0f;
    lastCmd.sideSpeed = 0.0f;
    lastCmd.rotateSpeed = 0.0f;
    lastCmd.bodyHeight = 0.0f;

    lastCmd.mode = 0;
    lastCmd.roll  = 0;
    lastCmd.pitch = 0;
    lastCmd.yaw = 0;
}

/** Updates lastCmd. */
void CmdClass::CmdCallback(unitree_legged_msgs::HighCmd newCmd)
{
    mutex.lock();
    lastCmd = newCmd;
    mutex.unlock();
}

/** Reads lastCmd. */
unitree_legged_msgs::HighCmd CmdClass::Read()
{
    boost::lock_guard<boost::mutex> lock{mutex};
    return lastCmd;
}

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
};

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);
    ros::Publisher pub_states = n.advertise<unitree_legged_msgs::HighState>("/a1_states", 500);

    CmdClass cmd_obj;
    ros::Subscriber sub_cmd = n.subscribe("/elevator/ctrl/highcmd", 10, &CmdClass::CmdCallback, &cmd_obj);

    // SetLevel(HIGHLEVEL);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd lastCmd;
    unitree_legged_msgs::HighState RecvHighROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    while (ros::ok()){
        ros::Time time = ros::Time::now();
        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        // RecvHighROS.stamp = time;
        pub_states.publish(RecvHighROS);

        lastCmd = cmd_obj.Read();
        // printf("%d %.3f %.3f\n",  lastCmd.mode, lastCmd.forwardSpeed, lastCmd.rotateSpeed);
        SendHighLCM = ToLcm(lastCmd, SendHighLCM);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "walk_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    if(firmwork == "3_1"){
        aliengo::Control control(aliengo::HIGHLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc, argv, roslcm);
    }
    else if(firmwork == "3_2"){
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

        UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    }

}