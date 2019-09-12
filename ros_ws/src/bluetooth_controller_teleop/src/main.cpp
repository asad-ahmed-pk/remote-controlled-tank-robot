// main.cpp
// Main node

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bluetooth_controller_teleop_node");
    ros::spin();

    return 0;
}