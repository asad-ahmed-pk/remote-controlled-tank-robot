//
// Created by Asad Ahmed on 19.09.19.
//

#include <ros/ros.h>
#include "joystick_controller/JoystickInputPublisher.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_controller");
    ros::NodeHandle nodeHandle("~");

    JoystickInputPublisher publisher(nodeHandle);
    while (ros::ok())
    {
        publisher.ListenForJoystickInput();
    }

    return 0;
}
