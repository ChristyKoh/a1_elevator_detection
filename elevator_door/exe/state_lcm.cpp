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

template <typename TLCM>
void *update_loop(void *param)
{
    TLCM *data = (TLCM *)param;
    while (ros::ok)
    {
        data->Recv();
        usleep(2000);
    }
}

template <typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    ros::NodeHandle nh;
    ros::Rate loop_rate(500);
    ros::Publisher pub_states = nh.advertise<unitree_legged_msgs::HighState>("/a1_states", 500);

    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighState RecvHighROS;
    roslcm.SubscribeState();
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    while (ros::ok())
    {
        ros::Time time = ros::Time::now();
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        RecvHighROS.stamp = time;
        pub_states.publish(RecvHighROS);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "state_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    if (firmwork == "3_1")
    {
        aliengo::Control control(aliengo::HIGHLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc, argv, roslcm);
    }
    else if (firmwork == "3_2")
    {
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if (strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if (strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

        UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    }
}
