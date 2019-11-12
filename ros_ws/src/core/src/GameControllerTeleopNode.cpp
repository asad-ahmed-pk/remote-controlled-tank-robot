//
// GameControllerTeleopNode.cpp
// ROS publisher node that reads input from the connected game controller node
// and publishes the input to /cmd_vel
//

#include <ros/ros.h>
#include <memory>

#include "joystick_msgs/JoystickInputMsg.h"
#include "geometry_msgs/Twist.h"

ros::Publisher publisher;

// joystick input callback
void JoystickInputCallback(const joystick_msgs::JoystickInputMsg& msg)
{

}

int main(int argc, char** argv)
{
    // init ROS node
    ros::init(argc, argv, "game_controller_teleop");
    ros::NodeHandle nodeHandle;

    // init publisher to publish onto cmd_vel
    publisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // the subscriber listening for the joystick messages
    ros::Subscriber subscriber = nodeHandle.subscribe("/joystick_input", 1, JoystickInputCallback);

    ros::spin();

    return 0;
}





