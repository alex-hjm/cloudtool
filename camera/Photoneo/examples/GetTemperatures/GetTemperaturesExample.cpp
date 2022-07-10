/*
* Photoneo's API Example - GetTemperaturesExample.cpp
* Prints sensor temperature values.
* Value: pho::api::PHOXI_DEVICE_TEMPERATURE_INVALID
*        -273.15 (temperature is not available)
*/

#include <vector>
#include <array>
#include <string>
#include <iostream>
#include <sstream>

#include "PhoXi.h"

//Print out list of device info to standard output
void printDeviceInfoList(const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList);
//Print out device info to standard output
void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo);
//Print out sensor temperatures
void printTemperatureValues(const pho::api::FrameInfo &FrameInfo);

int main(int argc, char *argv[])
{
    pho::api::PhoXiFactory Factory;

    //Check if the PhoXi Control Software is running
    if (!Factory.isPhoXiControlRunning())
    {
        std::cout << "PhoXi Control Software is not running" << std::endl;
        return 0;
    }

    //Get List of available devices on the network
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList = Factory.GetDeviceList();
    if (DeviceList.empty())
    {
        std::cout << "PhoXi Factory has found 0 devices" << std::endl;
        return 0;
    }
    printDeviceInfoList(DeviceList);

    //Try to connect device opened in PhoXi Control, if any
    pho::api::PPhoXi PhoXiDevice = Factory.CreateAndConnectFirstAttached();
    if (PhoXiDevice)
    {
        std::cout << "You have already PhoXi device opened in PhoXi Control, the API Example is connected to device: "
            << (std::string) PhoXiDevice->HardwareIdentification << std::endl;
    }
    else
    {
        std::cout << "You have no PhoXi device opened in PhoXi Control, the API Example will try to connect to first device in device list" << std::endl;
        PhoXiDevice = Factory.CreateAndConnect(DeviceList.front().HWIdentification);
    }

    //Check if device was created
    if (!PhoXiDevice)
    {
        std::cout << "Your device was not created!" << std::endl;
        return 0;
    }

    //Check if device is connected
    if (!PhoXiDevice->isConnected())
    {
        std::cout << "Your device is not connected" << std::endl;
        return 0;
    }
    std::cout << std::endl << std::endl;

    //Check if temperature reader is enabled
    PhoXiDevice->TemperaturesReader = pho::api::PhoXiTemperaturesReader::TempReaderAll;

    pho::api::PhoXiTemperaturesReader TempReader = PhoXiDevice->TemperaturesReader;
    if (TempReader != pho::api::PhoXiTemperaturesReader::TempReaderAll) {
        std::cout << "Temperature reader is not available for your setup!"<< std::endl;
        std::cout << "Update your Device or use newer API/PhoXiControl." << std::endl;
        return 0;
    }

    const int FrameID = PhoXiDevice->TriggerFrame();
    pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID);
    pho::api::FrameInfo &FrameInfo = Frame->Info;

    //Print out temperature values
    printTemperatureValues(FrameInfo);

    //Disconnect PhoXi device
    PhoXiDevice->Disconnect();
    return 0;
}

void printDeviceInfoList(const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList)
{
    for (std::size_t i = 0; i < DeviceList.size(); ++i)
    {
        std::cout << "Device: " << i << std::endl;
        printDeviceInfo(DeviceList[i]);
    }
}

void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo)
{
    std::cout << "  Name:                    " << DeviceInfo.Name << std::endl;
    std::cout << "  Hardware Identification: " << DeviceInfo.HWIdentification << std::endl;
    std::cout << "  Type:                    " << std::string(DeviceInfo.Type) << std::endl;
    std::cout << "  Firmware version:        " << DeviceInfo.FirmwareVersion << std::endl;
    std::cout << "  Status:                  "
        << (DeviceInfo.Status.Attached ? "Attached to PhoXi Control. " : "Not Attached to PhoXi Control. ")
        << (DeviceInfo.Status.Ready ? "Ready to connect" : "Occupied")
        << std::endl << std::endl;
}

void printTemperatureValues(const pho::api::FrameInfo &FrameInfo)
{
    std::cout << "Value: " << pho::api::PHOXI_DEVICE_TEMPERATURE_INVALID
        << " means that the temperature is not available." << std::endl;

    const auto TEMPS_COUNT = pho::api::TemperatureDeviceSensor::TempsCount;
    const std::array<double, TEMPS_COUNT> &Temps = FrameInfo.Temperatures;
    std::cout << "Temperatures are the following: " << std::endl;
    std::cout << "  ProjectionUnitBoard: "  << Temps[pho::api::TemperatureDeviceSensor::ProjectionUnitBoard] << std::endl;
    std::cout << "  ProjectionUnitLaser: "  << Temps[pho::api::TemperatureDeviceSensor::ProjectionUnitLaser] << std::endl;
    std::cout << "  ProjectionUnitLaser: "  << Temps[pho::api::TemperatureDeviceSensor::ProjectionUnitLaser] << std::endl;
    std::cout << "  CameraSensorBoard: "    << Temps[pho::api::TemperatureDeviceSensor::CameraSensorBoard] << std::endl;
    std::cout << "  CameraRearHousing: "    << Temps[pho::api::TemperatureDeviceSensor::CameraRearHousing] << std::endl;
    std::cout << "  CameraFrontHousing: "   << Temps[pho::api::TemperatureDeviceSensor::CameraFrontHousing] << std::endl;
    std::cout << "  CameraSensorDie: "      << Temps[pho::api::TemperatureDeviceSensor::CameraSensorDie] << std::endl;
    std::cout << "  CameraSensorDieRaw: "   << Temps[pho::api::TemperatureDeviceSensor::CameraSensorDieRaw] << std::endl;
    std::cout << "  CameraInterfaceBoard: " << Temps[pho::api::TemperatureDeviceSensor::CameraInterfaceBoard] << std::endl;
}
