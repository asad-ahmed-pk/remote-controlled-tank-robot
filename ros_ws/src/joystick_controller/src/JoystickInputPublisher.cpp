//
// Created by Asad Ahmed on 22.09.19.
//

#include "joystick_controller/JoystickInputPublisher.hpp"
#include "joystick_controller/JoystickInputHandler.hpp"

JoystickInputPublisher::JoystickInputPublisher(const ros::NodeHandle& nodeHandle) : m_NodeHandle(nodeHandle), m_Running(true)
{
    // load params from config file
    int joystickNumber = 0;
    if (!m_NodeHandle.getParam("controller/js_number", joystickNumber)) {
        ROS_ERROR("Failed to get config param: controller.js_number");
    }

    int controllerType = NINTENDO_SWITCH_PRO;
    if (!m_NodeHandle.getParam("controller/type", controllerType)) {
        ROS_ERROR("Failed to get config param: controller.type");
    }
    m_ControllerType = static_cast<ControllerType>(controllerType);

    // init joystick input handler
    m_InputHandler = JoystickInputHandler(joystickNumber);

    // message publication
    m_Publisher = m_NodeHandle.advertise<joystick_msgs::JoystickInputMsg>("/joystick_input", 1);
}

JoystickInputPublisher::~JoystickInputPublisher()
{

}

// Event listener
void JoystickInputPublisher::ListenForJoystickInput() const
{
    JoystickEvent e;
    joystick_msgs::JoystickInputMsg msg;

    if (m_InputHandler.ReadNextJoystickInput(e))
    {
        if (e.type != EVENT_OTHER) {
            ConstructMessageFromEvent(e, msg);
            m_Publisher.publish(msg);
        }
        else {
            PublishZeroInput();
        }
    }
    else {
        PublishZeroInput();
    }
}

// Publish zero input
void JoystickInputPublisher::PublishZeroInput() const
{
    // TODO: remove this hack - just publish axis with zero value input
    joystick_msgs::JoystickInputMsg msg;
    msg.input_type == joystick_msgs::JoystickInputMsg::INPUT_TYPE_AXIS;
    msg.value = 0;
    msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_TRIGGER_R2;

    m_Publisher.publish(msg);
}

// Message construction
void JoystickInputPublisher::ConstructMessageFromEvent(const JoystickEvent &e, joystick_msgs::JoystickInputMsg& msg) const
{
    // set axis or button value (-1 to 1)
    msg.value = e.value;

    if (e.type == EVENT_AXIS)
    {
        msg.input_type = joystick_msgs::JoystickInputMsg::INPUT_TYPE_AXIS;
    }
    else if (e.type == EVENT_BUTTON)
    {
        msg.input_type = joystick_msgs::JoystickInputMsg::INPUT_TYPE_BUTTON;
    }

    // map the correct button numbers based on the controller
    if (m_ControllerType == NINTENDO_SWITCH_PRO) {
        msg.controller_type = joystick_msgs::JoystickInputMsg::CONTROLLER_TYPE_SWITCH_PRO;
        MapNintendoSwitchProButtons(e, msg);
    }
    else if (m_ControllerType == PLAYSTATION_3) {
        msg.controller_type = joystick_msgs::JoystickInputMsg::CONTROLLER_TYPE_PS3;
        MapPS3Buttons(e, msg);
    }
}

// Maps nintendo switch controller input
void JoystickInputPublisher::MapNintendoSwitchProButtons(const JoystickEvent &e, joystick_msgs::JoystickInputMsg &msg) const
{
    // mapping axis
    if (e.type == EVENT_AXIS)
    {
        if (e.number == 0) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_LEFT_STICK_X;
        }
        else if (e.number == 3) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_RIGHT_STICK_X;
        }
        else if (e.number == 2) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_TRIGGER_L2;
        }
        else if (e.number == 5) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_TRIGGER_R2;
        }
        else if (e.number == 6) {
            msg.number = e.value == 1.0 ? joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D_PAD_RIGHT : joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D_PAD_LEFT;
        }
    }
    else if (e.type == EVENT_BUTTON)
    {
        // mapping button event
        if (e.number == 3) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_A;
        }
        else if (e.number == 1) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_B;
        }
        else if (e.number == 0) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_C;
        }
        else if (e.number == 2) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D;
        }
        else if (e.number == 7) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_START;
        }
        else if (e.number == 6) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_SELECT;
        }
        else if (e.number == 8) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_HOME;
        }
    }
}

// Map PS3 controller
void JoystickInputPublisher::MapPS3Buttons(const JoystickEvent &e, joystick_msgs::JoystickInputMsg &msg) const
{
    if (e.type == EVENT_AXIS)
    {
        // mapping PS3 axis event
        if (e.number == 0) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_LEFT_STICK_X;
        }
        else if (e.number == 1) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_LEFT_STICK_Y;
        }
        else if (e.number == 3) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_RIGHT_STICK_X;
        }
        else if (e.number == 4) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_RIGHT_STICK_Y;
        }
        else if (e.number == 2) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_TRIGGER_R2;
        }
        else if (e.number == 5) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_TRIGGER_L2;
        }
    }
    else if (e.type == EVENT_BUTTON)
    {
        // mapping PS3 button event
        if (e.number == 2) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_A;
        }
        else if (e.number == 1) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_B;
        }
        else if (e.number == 0) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_C;
        }
        else if (e.number == 3) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D;
        }
        else if (e.number == 9) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_START;
        }
        else if (e.number == 8) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_SELECT;
        }
        else if (e.number == 10) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_HOME;
        }
        else if (e.number == 5) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_TRIGGER_R1;
        }
        else if (e.number == 4) {
            msg.number = joystick_msgs::JoystickInputMsg::AXIS_TYPE_TRIGGER_L1;
        }
        else if (e.number == 13) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D_PAD_UP;
        }
        else if (e.number == 14) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D_PAD_DOWN;
        }
        else if (e.number == 15) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D_PAD_LEFT;
        }
        else if (e.number == 16) {
            msg.number = joystick_msgs::JoystickInputMsg::BUTTON_TYPE_D_PAD_RIGHT;
        }
    }
}
