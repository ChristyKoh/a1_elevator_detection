# a1_elevator_detection
Elevator detection for the Unitree A1 quadruped robot using the Intel D435 RealSense camera.
For a high-level overview, please see [106A Final Project Website](https://sites.google.com/berkeley.edu/robotguidedog-multifloornav/implementation?authuser=0#h.av35uhnwuwma).

## Running Code
Start the Realsense camera.
```
roslaunch realsense2_camera rs_camera.launch enable_pointcloud:=true enable_color:=true enable_depth:=true initial_reset:=true
```
Start the LCM to ROS message server as super user.
```
su
source ~/guidedog_ws/devel/setup.bash
rosrun elevator_door state_cmd_lcm
```
Start the door detection and floor estimation code in separate terminal windows.
```
rosrun elevator_door elevator_detection.py
rosrun elevator_floor elevator_floor_main.py
```
To make the robot move, in another window, run the state machine code.
```
rosrun elevator_door elevator_control.py
```
