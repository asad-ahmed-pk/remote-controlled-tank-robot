// js_test.cpp

#include <unistd.h>
#include <iostream>

#include "joystick_controller/JoystickInputHandler.hpp"

int main(int argc, char** argv)
{
    JoystickInputHandler handler { 0 };
    JoystickEvent e;

    while (true)
    {
        if (handler.ReadNextJoystickInput(e))
        {
            if (e.type == EVENT_BUTTON) {
                std::cout << "Button " << e.number << ": ";
            } else if (e.type == EVENT_AXIS) {
                std::cout << "Axis " << e.number << ": ";
            } else {
                std::cout << "Other" << std::endl;
                continue;
            }

            std::cout << e.value << std::endl;
        }

        usleep(1000);
    }
}