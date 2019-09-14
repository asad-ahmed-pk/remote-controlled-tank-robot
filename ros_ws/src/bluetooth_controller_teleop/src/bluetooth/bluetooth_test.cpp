// bluetooth_test.cpp

#include "BluetoothManager.hpp"

#include <iostream>
#include <vector>
#include <tuple>
#include <string>

const std::string PRO_CONTROLLER_ADDRESS { "B8:27:EB:35:A1:F4" };

// callback
void BluetoothCallback(const std::string& data);

int main(int argc, char** argv)
{
    BluetoothManager manager;

    std::vector<std::tuple<std::string, std::string>> devices;

    std::cout << "\nScanning for devices" << std::endl;

    // scan devices
    if (manager.ScanDevices(devices))
    {
        for (const std::tuple<std::string, std::string>& device : devices) {
            std::cout << "\nAddress: " << std::get<0>(device);
            std::cout << " Name: " << std::get<1>(device) << std::endl;
        }

        if (devices.empty()) {
            std::cout << "\nCould not find any devices in range" << std::endl;
        }
    }
    else {
        std::cout << "Failed to connect to bluetooth adapter" << std::endl;
    }

    // read from pro controller
    std::cout << "\nConnecting to Pro Controller" << std::endl;
    if (!manager.ProcessInput(PRO_CONTROLLER_ADDRESS, BluetoothCallback)) {
        std::cout << "Could not connect to pro controller" << std::endl;
        return 1;
    }

    return 0;
}

void BluetoothCallback(const std::string& data)
{
    std::cout << "Data: " << data << std::endl;
}