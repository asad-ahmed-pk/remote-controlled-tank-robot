//
// Created by asad on 14.09.19.
//

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <sys/socket.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <rfcomm.h>

#include "BluetoothManager.hpp"

const int MAX_RSP = 255;
const int FLAGS = IREQ_CACHE_FLUSH;
const int QUERY_LEN = 8;

const int ADDRESS_BUFFER_SIZE = 19;
const int DEVICE_NAME_BUFFER_SIZE = 256;

const int READ_BUFFER_SIZE = 256;

BluetoothManager::BluetoothManager() {

}

BluetoothManager::~BluetoothManager() {

}

// Network scan
bool BluetoothManager::ScanDevices(std::vector<std::tuple<std::string, std::string>>& devices)
{
    std::unique_ptr<inquiry_info[]> inquiries;

    int numDevices = 0;
    int socketID = 0;
    int localDeviceID = 0;

    char address[ADDRESS_BUFFER_SIZE] = { 0 };
    char name[DEVICE_NAME_BUFFER_SIZE] = { 0 };

    localDeviceID = hci_get_route(nullptr);
    socketID = hci_open_dev(localDeviceID);

    // error check in connecting to local bluetooth adapter
    if (localDeviceID < 0 || socketID < 0) {
        return false;
    }

    // connected to local adapter - begin device query
    inquiries = std::unique_ptr<inquiry_info[]>(new inquiry_info[MAX_RSP]);
    numDevices = hci_inquiry(localDeviceID, QUERY_LEN, MAX_RSP, nullptr, reinterpret_cast<inquiry_info **>(&inquiries), FLAGS);

    std::string deviceAddress;
    std::string deviceName;

    for (int i = 0; i < numDevices; i++)
    {
        // get remote device address
        bdaddr_t bdaddr = inquiries.get()[i].bdaddr;
        ba2str(&bdaddr, address);
        deviceAddress = std::move(std::string(address));

        // get remote device name
        if (hci_read_remote_name(socketID, &bdaddr, sizeof(name), name, 0) < 0) {
            deviceName = "Unknown Device";
        }
        else {
            deviceName = std::move(std::string(name));
        }

        auto tupleItem = std::tuple<std::string, std::string>(deviceAddress, deviceName);
        devices.emplace_back(tupleItem);
    }

    return true;
}

// Read input from bluetooth device and call callback whenever input is received
bool BluetoothManager::ProcessInput(const std::string &deviceAddress, const std::function<void(const std::string&)> &callback)
{
    sockaddr_rc address = { 0 };
    int s = 0, status = 0, len = 0;

    char buffer[READ_BUFFER_SIZE];

    // allocate socket for reading from device
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // connect to given device with address on bluetooth
    address.rc_family = AF_BLUETOOTH;
    address.rc_channel = (uint8_t) 1;
    str2ba(deviceAddress.c_str(), &address.rc_bdaddr);

    status = connect(s, (sockaddr *)&address, sizeof(address));
    if (status) {
        return false;
    }

    // run loop processing input (blocking)
    std::string data;
    len = read(s, buffer, sizeof(buffer));

    while (len > 0)
    {
        if (len > 0) {
            data = std::move(std::string(buffer));
            callback(data);
        }

        len = read(s, buffer, sizeof(buffer));
    }

    // close and clean up
    close(s);

    return true;
}
