//
// Created by Asad Ahmed on 22.09.19.
//
// Publisher responsible for reading joystick input and publishing ROS messages
//

#ifndef JOYSTICK_CONTROLLER_JOYSTICKINPUTPUBLISHER_H
#define JOYSTICK_CONTROLLER_JOYSTICKINPUTPUBLISHER_H

#include <ros/ros.h>

#include "joystick_controller/JoystickInputHandler.hpp"
#include "joystick_msgs/JoystickInputMsg.h"

class JoystickInputPublisher
{
public:
    JoystickInputPublisher(const ros::NodeHandle& nodeHandle);
    ~JoystickInputPublisher();

    /**
     * Listens to input events from the joystick and published ROS messages
     */
    void ListenForJoystickInput() const;

private:
    void ConstructMessageFromEvent(const JoystickEvent& e, joystick_msgs::JoystickInputMsg& msg) const;
    void MapNintendoSwitchProButtons(const JoystickEvent& e, joystick_msgs::JoystickInputMsg& msg) const;
    void MapPS3Buttons(const JoystickEvent& e, joystick_msgs::JoystickInputMsg& msg) const;
    void PublishZeroInput() const;

private:
    ros::NodeHandle m_NodeHandle;
    ros::Publisher m_Publisher;
    JoystickInputHandler m_InputHandler;
    ControllerType m_ControllerType;
    bool m_Running;
};

#endif //JOYSTICK_CONTROLLER_JOYSTICKINPUTPUBLISHER_H
