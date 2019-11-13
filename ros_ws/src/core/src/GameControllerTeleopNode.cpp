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
    int x = 0;
    int z = 0;
    
    // Emergency stop button
    if (msg.input_type == joystick_msgs::JoystickInputMsg::INPUT_TYPE_BUTTON)
    {
        if (msg.number == joystick_msgs::JoystickInputMsg::BUTTON_TYPE_C) {
            x = 0;
            z = 0;
        }
    }

    // D-Pad controls
    /*
    if (msg.input_type == joystick_msgs::JoystickInputMsg::INPUT_TYPE_BUTTON)
    {
        switch (msg.number)
        {
            case joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D_PAD_UP:
                z = 1;
                break;

            case joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D_PAD_DOWN:
                z = -1;
                break;

            case joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D_PAD_LEFT:
                x = -1;
                break;

            case joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D_PAD_RIGHT:
                x = 1;
                break;

            default:
                break;
        }
    }
    */
    // Axis controls
    else if (msg.input_type == joystick_msgs::JoystickInputMsg::INPUT_TYPE_AXIS)
    {
        switch (msg.number)
        {
            case joystick_msgs::JoystickInputMsg::AXIS_TYPE_LEFT_STICK_Y:
                z = -1 * msg.value;
                x = 0;
                break;
                
            case joystick_msgs::JoystickInputMsg::AXIS_TYPE_RIGHT_STICK_X:
                x = -1 * msg.value;
                z = 0;
                break;
        }
    }
         

    // publish the cmd_vel
    geometry_msgs::Twist vel;
    vel.linear.x = x;
    vel.linear.z = z;

    publisher.publish(vel);
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





