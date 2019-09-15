//
// Created by asad on 14.09.19.
//

#ifndef BLUETOOTH_CONTROLLER_TELEOP_BLUETOOTHMANAGER_HPP
#define BLUETOOTH_CONTROLLER_TELEOP_BLUETOOTHMANAGER_HPP

#include <tuple>
#include <vector>
#include <string>
#include <functional>

class BluetoothManager
{
public:
    BluetoothManager();
    ~BluetoothManager();

    /**
     * Perform a scan of the bluetooth devices in proximity of the host
     * @param devices Tuple vector that will be set with the device info: 0: ID, 1: the name of the device
     */
    bool ScanDevices(std::vector<std::tuple<std::string, std::string>>& devices) const;

    bool ProcessInput(const std::string& deviceAddress, const std::function<void(const std::string&)>& callback) const;

};


#endif //BLUETOOTH_CONTROLLER_TELEOP_BLUETOOTHMANAGER_HPP
