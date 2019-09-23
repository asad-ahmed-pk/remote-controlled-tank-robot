// JoystickInputHandler.cpp

#include <string>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>

#include "joystick_controller/JoystickInputHandler.hpp"

const signed short AXIS_RAW_MIN_VALUE { -32767 };
const signed short AXIS_RAW_MAX_VALUE { 32767 };

const float AXIS_MIN_VALUE { -1.0 };
const float AXIS_MAX_VALUE { 1.0 };

const std::string JS_FILE_PATH { "/dev/input/js" };

JoystickInputHandler::JoystickInputHandler(int joystickNumber) : m_JSFileDescriptor(-1), m_JoystickNumber(joystickNumber)
{
    // open file descriptor for js number
    std::string filePath = JS_FILE_PATH + std::to_string(m_JoystickNumber);
    m_JSFileDescriptor = open(filePath.c_str(), O_RDONLY);
}

JoystickInputHandler::~JoystickInputHandler() {}

bool JoystickInputHandler::ReadNextJoystickInput(JoystickEvent& event) const
{
    js_event e;
    if (m_JSFileDescriptor >= 0)
    {
        int bytes = read(m_JSFileDescriptor, &e, sizeof(e));
        if (bytes > 0)
        {
            // pack input into input struct
            GetInputType(e, event.type);

            if (event.type == EVENT_AXIS) {
                event.value = MapAxisValue(static_cast<float>(e.value));
            }
            else if (event.type == EVENT_BUTTON) {
                event.value = static_cast<float>(e.value);
            }
            else {
                event.value = 0.0;
            }

            event.number = e.number;

            return true;
        }
    }

    return false;
}

// Determines the type of input based on the OS event
void JoystickInputHandler::GetInputType(const js_event &e, JoystickInputType &inputType) const
{
    if (e.type == JS_EVENT_BUTTON) {
        inputType = EVENT_BUTTON;
    }
    else if (e.type == JS_EVENT_AXIS) {
        inputType = EVENT_AXIS;
    }
    else {
        inputType = EVENT_OTHER;
    }
}

// Maps the axis value to a range of -1 and 1
float JoystickInputHandler::MapAxisValue(float rawValue) const
{
    float inputMin = static_cast<float>(AXIS_RAW_MIN_VALUE);
    float inputMax = static_cast<float>(AXIS_RAW_MAX_VALUE);

    float value = AXIS_MIN_VALUE + ((AXIS_MAX_VALUE - AXIS_MIN_VALUE) / (inputMax - inputMin)) * (rawValue - inputMin);

    return value;
}