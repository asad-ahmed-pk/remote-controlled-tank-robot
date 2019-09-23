// JoystickInputHandler.hpp

#ifndef __JOYSTICK_INPUT_HANDLER_HPP__
#define __JOYSTICK_INPUT_HANDLER_HPP__

#include <linux/joystick.h>

// Represents the type of controller
enum ControllerType {
    NINTENDO_SWITCH_PRO,
    PLAYSTATION_3
};

// Represents a type of input (button or axis)
enum JoystickInputType {
    EVENT_BUTTON,
    EVENT_AXIS,
    EVENT_OTHER
};

// Represents the joystick event
struct JoystickEvent {
    float value;
    unsigned int number;
    JoystickInputType type;
};

class JoystickInputHandler
{
public:
    explicit JoystickInputHandler(int joystickNumber = 0);
    ~JoystickInputHandler();

    // Read the next joystick event input and store in event
    bool ReadNextJoystickInput(JoystickEvent& event) const;

private:
    void GetInputType(const js_event& e, JoystickInputType& inputType) const;
    float MapAxisValue(float rawValue) const;

private:
    int m_JSFileDescriptor;
    int m_JoystickNumber;
};

#endif