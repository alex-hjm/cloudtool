/*
* Photoneo's API Example - GetProfilesExample.cpp
* Prints profiles list.
* Get active profile
* Set active profile
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
//Print out profiles
void printProfilesList(const std::vector<pho::api::PhoXiProfileDescriptor> &ProfilesInfo);

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

    if (!PhoXiDevice->Profiles.isEnabled())
    {
        std::cout << "Your device does not support profiles" << std::endl;
        return 0;
    }

    std::vector<pho::api::PhoXiProfileDescriptor> ProfilesList = PhoXiDevice->Profiles;
    if (!PhoXiDevice->Profiles.isLastOperationSuccessful())
    {
        std::cout << "Can not get profile list: " << PhoXiDevice->Profiles.GetLastErrorMessage() << std::endl;
        return 0;
    }

    // Print out profiles
    printProfilesList(ProfilesList);

    std::string ActiveProfile = PhoXiDevice->ActiveProfile;
    if (!PhoXiDevice->ActiveProfile.isLastOperationSuccessful())
    {
        std::cout << "Can not get active profile: " << PhoXiDevice->ActiveProfile.GetLastErrorMessage() << std::endl;
        return 0;
    }

    // get any profile different from active profile
    std::string NewActiveProfile;
    for (const pho::api::PhoXiProfileDescriptor &Profile : ProfilesList)
    {
        if (Profile.Name != ActiveProfile) {
            NewActiveProfile = Profile.Name;
            break;
        }
    }

    // set new active profile
    PhoXiDevice->ActiveProfile = NewActiveProfile;
    if (PhoXiDevice->ActiveProfile.isLastOperationSuccessful())
    {
        std::cout << "Active profile set to : " << NewActiveProfile << std::endl;
    }
    else
    {
        std::cout << "Can not set active profile: " << PhoXiDevice->ActiveProfile.GetLastErrorMessage() << std::endl;
        return 0;
    }

    // export profile from device
    pho::api::PhoXiProfileContent ProfileContent = PhoXiDevice->ExportProfile;
    if (PhoXiDevice->ExportProfile.isLastOperationSuccessful())
    {
        std::cout << "Active profile was exported " << std::endl;
    }
    else
    {
        std::cout << "Can not export active profile: " << PhoXiDevice->ActiveProfile.GetLastErrorMessage() << std::endl;
        return 0;
    }

    //rename profile and import to device
    ProfileContent.Name = "My Profile";
    PhoXiDevice->ImportProfile = ProfileContent;
    if (PhoXiDevice->ImportProfile.isLastOperationSuccessful())
    {
        std::cout << "Profile with name: " <<  ProfileContent.Name << " was imported." << std::endl;
    }
    else
    {
        std::cout << "Can not import profile: " << PhoXiDevice->ImportProfile.GetLastErrorMessage() << std::endl;
        return 0;
    }

    PhoXiDevice->CreateProfile = "MyProfile";
    if (PhoXiDevice->CreateProfile.isLastOperationSuccessful())
    {
        std::cout << "Profile was created." << std::endl;
    }
    else
    {
        std::cout << "Can not create profile: " << PhoXiDevice->ImportProfile.GetLastErrorMessage() << std::endl;
        return 0;
    }

    PhoXiDevice->UpdateProfile = "MyProfile";
    if (PhoXiDevice->UpdateProfile.isLastOperationSuccessful())
    {
        std::cout << "Profile was updated." << std::endl;
    }
    else
    {
        std::cout << "Can not update profile: " << PhoXiDevice->ImportProfile.GetLastErrorMessage() << std::endl;
        return 0;
    }

    PhoXiDevice->DeleteProfile = "MyProfile";
    if (PhoXiDevice->DeleteProfile.isLastOperationSuccessful())
    {
        std::cout << "Profile was deleted." << std::endl;
    }
    else
    {
        std::cout << "Can not delete profile: " << PhoXiDevice->ImportProfile.GetLastErrorMessage() << std::endl;
        return 0;
    }
    // Disconnect PhoXi device
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

void printProfilesList(const std::vector<pho::api::PhoXiProfileDescriptor> &ProfilesList) {
    std::cout << std::boolalpha << true;
    for (const pho::api::PhoXiProfileDescriptor &profile : ProfilesList)
    {
        std::cout << "Profile: " << std::endl;
        std::cout << "  Name: "<< profile.Name << std::endl;
        std::cout << "  Is factory profile: " << profile.IsFactory << std::endl;
    }
}

