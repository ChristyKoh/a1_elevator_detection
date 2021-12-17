#ifndef CMD_H
#define CMD_H

#include <unitree_legged_msgs/HighCmd.h>

class CmdClass {
    public:
        CmdClass();

        void CmdCallback(unitree_legged_msgs::HighCmd);
        
        unitree_legged_msgs::HighCmd Read();

        unitree_legged_msgs::HighCmd lastCmd;
        boost::mutex mutex;
};

#endif