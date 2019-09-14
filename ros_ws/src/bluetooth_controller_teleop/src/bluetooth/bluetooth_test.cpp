// bluetooth_test.cpp

#include "BluetoothManager.hpp"

#include <iostream>
#include <vector>
#include <tuple>
#include <string>

int main(int argc, char** argv)
{
    std::vector<std::tuple<std::string, std::string>> devices;

    BluetoothManager manager;
    if (manager.ScanDevices(devices))
    {
        for (const std::tuple<std::string, std::string>& device : devices) {
            std::cout << "\nAddress: " << std::get<0>(device);
            std::cout << " Name: " << std::get<1>(device) << std::endl;
        }
    }
    else {
        std::cout << "Failed to connected to bluetooth adapter" << std::endl;
    }

    return 0;
}