/*
* Photoneo's API Example - FullAPIExample.cpp
* Defines the entry point for the console application.
* Demonstrates the extended functionality of PhoXi devices. This Example shows various ways how to connect to device.
* Contains the usage of retrieving all parameters of the device. Tests the software trigger and free run mode.
* Describes the exact steps needed to change device's settings, to handle and save received frame.
* Points out the correct way to disconnect the device from PhoXiControl.
*/

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <thread>
#include <atomic>
#if defined(_WIN32)
    #include <windows.h>
#elif defined (__linux__)
    #include <unistd.h>
#endif

#include "PhoXi.h"

#if defined(_WIN32)
    #define LOCAL_CROSS_SLEEP(Millis) Sleep(Millis)
    #define DELIMITER "\\"
#elif defined (__linux__) || defined(__APPLE__)
    #define LOCAL_CROSS_SLEEP(Millis) usleep(Millis * 1000)
    #define DELIMITER "/"
#endif

//The whole api is in namespace pho (Photoneo) :: api
class FullAPIExample
{
  private:
    pho::api::PhoXiFactory Factory;
    pho::api::PPhoXi PhoXiDevice;
    pho::api::PFrame SampleFrame;
    std::vector <pho::api::PhoXiDeviceInformation> DeviceList;
    std::string FileCameraFolder = "";
    std::string OutputFolder = "";

    void GetAvailableDevicesExample();
    void ConnectPhoXiDeviceExample();
    void ConnectPhoXiDeviceBySerialExample();
    void ConnectPhoXiDeviceByPhoXiDeviceInformationEntryExample();
    void ConnectFirstAttachedPhoXiDeviceExample();
    void ConnectPhoXiFileCameraExample();
    void BasicDeviceStateExample();
    void FreerunExample();
    void SoftwareTriggerExample();
    void SoftwareTriggerAsyncGrabExample();
    void ChangeSettingsExample();
    void DataHandlingExample();
    void CorrectDisconnectExample();

    void PrintFrameInfo(const pho::api::PFrame &Frame);
    void PrintFrameData(const pho::api::PFrame &Frame);
    void PrintCapturingSettings(const pho::api::PhoXiCapturingSettings &CapturingSettings);
    void PrintProcessingSettings(const pho::api::PhoXiProcessingSettings &ProcessingSettings);
    void PrintCoordinatesSettings(const pho::api::PhoXiCoordinatesSettings &CoordinatesSettings);
    void PrintCalibrationSettings(const pho::api::PhoXiCalibrationSettings &CalibrationSettings);
    void PrintMatrix(const std::string &name, const pho::api::CameraMatrix64f &matrix);
    void PrintVector(const std::string &name, const pho::api::Point3_64f &vector);

    template<class T>
    bool ReadLine(T &Output) const
    {
        std::string Input;
        std::getline(std::cin, Input);
        std::stringstream InputSteam(Input);
        return (InputSteam >> Output) ? true : false;
    }
    bool ReadLine(std::string &Output) const
    {
        std::getline(std::cin, Output);
        return true;
    }

  public:
    FullAPIExample() {};
    ~FullAPIExample() {};
    void Run();
};

void FullAPIExample::GetAvailableDevicesExample()
{
    //Wait for the PhoXiControl
    while (!Factory.isPhoXiControlRunning())
    {
        LOCAL_CROSS_SLEEP(100);
    }
    std::cout << "PhoXi Control Version: " << Factory.GetPhoXiControlVersion() << std::endl;
    std::cout << "PhoXi API Version: " << Factory.GetAPIVersion() << std::endl;

    DeviceList = Factory.GetDeviceList();
    std::cout << "PhoXi Factory found " << DeviceList.size() << " devices." << std::endl << std::endl;
    pho::api::PhoXiDeviceInformation *DeviceInfo;
    for (std::size_t i = 0; i < DeviceList.size(); ++i)
    {
        DeviceInfo = &DeviceList[i];
        std::cout << "Device: " << i << std::endl;
        std::cout << "  Name:                    " << DeviceInfo->Name << std::endl;
        std::cout << "  Hardware Identification: " << DeviceInfo->HWIdentification << std::endl;
        std::cout << "  Type:                    " << std::string(DeviceInfo->Type) << std::endl;
        std::cout << "  Firmware version:        " << DeviceInfo->FirmwareVersion << std::endl;
        std::cout << "  Variant:                 " << DeviceInfo->Variant << std::endl;
        std::cout << "  IsFileCamera:            " << (DeviceInfo->IsFileCamera ? "Yes" : "No") << std::endl;
        std::cout << "  Status:                  "
            << (DeviceInfo->Status.Attached ? "Attached to PhoXi Control. " : "Not Attached to PhoXi Control. ")
            << (DeviceInfo->Status.Ready    ? "Ready to connect"            : "Occupied")
            << std::endl << std::endl;
    }
}

void FullAPIExample::ConnectPhoXiDeviceExample()
{
    //You can connect to any device connected to local network (with compatible IP4 settings)
    //The connection can be made in multiple ways
    for (;;)
    {
        std::cout << "Please enter the number of the way to connect to your device from this possibilities:" << std::endl;
        std::cout << "  1. Connect by Hardware Identification Number" << std::endl;
        std::cout << "  2. Connect by Index listed from GetDeviceList call" << std::endl;
        std::cout << "  3. Connect first device Attached to PhoXi Control - if any" << std::endl;
        std::cout << "  4. Connect to file camera in folder: " << FileCameraFolder << std::endl << std::endl;
        std::cout << "  5. Refresh GetDeviceList" << std::endl << std::endl;
        std::cout << "Please enter the choice: ";

        std::size_t Index;
        if (!ReadLine(Index))
        {
            std::cout << "Incorrect input!" << std::endl;
            continue;
        }

        switch (Index)
        {
            case 1:
                ConnectPhoXiDeviceBySerialExample();
                break;
            case 2:
                ConnectPhoXiDeviceByPhoXiDeviceInformationEntryExample();
                break;
            case 3:
                ConnectFirstAttachedPhoXiDeviceExample();
                break;
            case 4:
                ConnectPhoXiFileCameraExample();
                break;
            case 5:
                GetAvailableDevicesExample();
                break;
            default:
                continue;
        }
        if (PhoXiDevice && PhoXiDevice->isConnected())
        {
            std::cout << "You are connected to " << std::string(PhoXiDevice->GetType())
                << " with Hardware Identification " << std::string(PhoXiDevice->HardwareIdentification)
                << std::endl;
            break;
        }
    }
}

void FullAPIExample::ConnectPhoXiDeviceBySerialExample()
{
    std::cout << std::endl << "Please enter the Hardware Identification Number: ";
    std::string HardwareIdentification;
    if (!ReadLine(HardwareIdentification))
    {
        std::cout << "Incorrect input!" << std::endl;
        return;
    }

    pho::api::PhoXiTimeout Timeout = pho::api::PhoXiTimeout::ZeroTimeout;
    PhoXiDevice = Factory.CreateAndConnect(HardwareIdentification, Timeout);
    if (PhoXiDevice)
    {
        std::cout << "Connection to the device " << HardwareIdentification << " was Successful!" << std::endl;
    }
    else
    {
        std::cout << "Connection to the device " << HardwareIdentification << " was Unsuccessful!" << std::endl;
    }
}

void FullAPIExample::ConnectPhoXiDeviceByPhoXiDeviceInformationEntryExample()
{
    std::cout << std::endl << "Please enter the Index listed from GetDeviceList call: ";

    std::size_t Index;
    if (!ReadLine(Index))
    {
        std::cout << "Incorrect input!" << std::endl;
        return;
    }

    if (Index >= DeviceList.size())
    {
        std::cout << "Bad Index, or not number!" << std::endl;
        return;
    }

    PhoXiDevice = Factory.Create(DeviceList[Index]);
    if (!PhoXiDevice)
    {
        std::cout << "Device " << DeviceList[Index].HWIdentification << " was not created" << std::endl;
        return;
    }

    if (PhoXiDevice->Connect())
    {
        std::cout << "Connection to the device " << DeviceList[Index].HWIdentification << " was Successful!" << std::endl;
    }
    else
    {
        std::cout << "Connection to the device " << DeviceList[Index].HWIdentification << " was Unsuccessful!" << std::endl;
    }
}

void FullAPIExample::ConnectFirstAttachedPhoXiDeviceExample()
{
    PhoXiDevice = Factory.CreateAndConnectFirstAttached();
    if (PhoXiDevice)
    {
        std::cout << "Connection to the device " << std::string(PhoXiDevice->HardwareIdentification) << " was Successful!" << std::endl;
    }
    else
    {
        std::cout << "There is no attached device, or the device is not ready!" << std::endl;
    }
}

void FullAPIExample::ConnectPhoXiFileCameraExample()
{
    std::vector<std::string> prawFolder { FileCameraFolder };
    const auto name = "TestFileCamera";
    const auto fileCameraName = Factory.AttachFileCamera(name, prawFolder);

    if (fileCameraName.empty())
    {
        std::cout << "Could not create file camera! Check whether praw files are in the specified folder: " << prawFolder[0] << std::endl;
        return;
    }

    PhoXiDevice = Factory.CreateAndConnect(fileCameraName, pho::api::PhoXiTimeout::Infinity);
    if (PhoXiDevice)
    {
        std::cout << "Connection to the device " << static_cast<std::string>(PhoXiDevice->HardwareIdentification) << " was Successful!" << std::endl;
        // In file camera you can't change settings thus we stop the program flow
        CorrectDisconnectExample();
    }
    else
    {
        std::cout << "There is no attached device, or the device is not ready!" << std::endl;
    }
}

void FullAPIExample::BasicDeviceStateExample()
{
    //Check if the device is connected
    if (!PhoXiDevice || !PhoXiDevice->isConnected())
    {
        std::cout << "Device is not created, or not connected!" << std::endl;
        return;
    }

    std::cout << "  Status:" << std::endl;
    std::cout << "    "
        << (PhoXiDevice->isConnected() ? "Device is connected" : "Device is not connected (Error)")
        << std::endl;
    std::cout << "    "
        << (PhoXiDevice->isAcquiring() ? "Device is in acquisition mode" : "Device is not in acquisition mode")
        << std::endl;

    std::vector <std::string> SupportedFeatures = PhoXiDevice->Features.GetSupportedFeatures();
    std::cout << "  This device have these features supported:";
    for (std::size_t i = 0; i < SupportedFeatures.size(); ++i)
    {
        std::cout << " " << SupportedFeatures[i] << ";";
    }
    std::cout << std::endl << std::endl;

    //We will go trough all current Device features
    //You can ask the feature if it is implemented and if it is possible to Get or Set the feature value
    if (PhoXiDevice->HardwareIdentification.isEnabled() && PhoXiDevice->HardwareIdentification.CanGet())
    {
        std::string HardwareIdentification = PhoXiDevice->HardwareIdentification;
        if (!PhoXiDevice->HardwareIdentification.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->HardwareIdentification.GetLastErrorMessage().c_str());
        }
        std::cout << "  HardwareIdentification: " << HardwareIdentification << std::endl;
    }

    //PhoXiCapturingMode
    if (PhoXiDevice->CapturingMode.isEnabled() && PhoXiDevice->CapturingMode.CanGet())
    {
        pho::api::PhoXiCapturingMode CapturingMode = PhoXiDevice->CapturingMode;
        //You can ask the feature, if the last performed operation was successful
        if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());
        }
        pho::api::PhoXiSize Resolution = CapturingMode.Resolution;
        //You can also access the resolution by PhoXiDevice->Resolution;
        std::cout << "  CapturingMode: " << std::endl;
        std::cout << "    Resolution:" << std::endl;
        std::cout << "      Width: "   << Resolution.Width << std::endl; /*PhoXiDevice->Resolution->Width*/
        std::cout << "      Height: "  << Resolution.Height << std::endl; /*PhoXiDevice->Resolution->Height*/
    }

    //PhoXiCapturingMode
    if (PhoXiDevice->SupportedCapturingModes.isEnabled() && PhoXiDevice->SupportedCapturingModes.CanGet())
    {
        std::vector <pho::api::PhoXiCapturingMode> SupportedCapturingModes = PhoXiDevice->SupportedCapturingModes;
        if (!PhoXiDevice->SupportedCapturingModes.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->SupportedCapturingModes.GetLastErrorMessage().c_str());
        }
        std::cout << "  SupportedCapturingModes: " << std::endl;
        for (std::size_t i = 0; i < SupportedCapturingModes.size(); ++i)
        {
            std::cout << "    (" 
                << std::to_string(SupportedCapturingModes[i].Resolution.Width) << " x "
                << std::to_string(SupportedCapturingModes[i].Resolution.Height) << ")"
                << std::endl;
        }
    }

    //PhoXiTriggerMode
    if (PhoXiDevice->TriggerMode.isEnabled() && PhoXiDevice->TriggerMode.CanGet())
    {
        pho::api::PhoXiTriggerMode TriggerMode = PhoXiDevice->TriggerMode;
        if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
        }
        std::cout << "  TriggerMode: " << std::string(TriggerMode) << std::endl;
    }

    //PhoXiTimeout
    if (PhoXiDevice->Timeout.isEnabled() && PhoXiDevice->Timeout.CanGet())
    {
        pho::api::PhoXiTimeout Timeout = PhoXiDevice->Timeout;
        if (!PhoXiDevice->Timeout.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->Timeout.GetLastErrorMessage().c_str());
        }
        std::cout << "  Timeout: " << std::string(Timeout) << std::endl;
    }

    //PhoXiCapturingSettings
    if (PhoXiDevice->CapturingSettings.isEnabled() && PhoXiDevice->CapturingSettings.CanGet())
    {
        pho::api::PhoXiCapturingSettings CapturingSettings = PhoXiDevice->CapturingSettings;
        if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->CapturingSettings.GetLastErrorMessage().c_str());
        }
        PrintCapturingSettings(CapturingSettings);
    }

    //PhoXiProcessingSettings
    if (PhoXiDevice->ProcessingSettings.isEnabled() && PhoXiDevice->ProcessingSettings.CanGet())
    {
        pho::api::PhoXiProcessingSettings ProcessingSettings = PhoXiDevice->ProcessingSettings;
        if (!PhoXiDevice->ProcessingSettings.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->ProcessingSettings.GetLastErrorMessage().c_str());
        }
        PrintProcessingSettings(ProcessingSettings);
    }

    //PhoXiCoordinatesSettings
    if (PhoXiDevice->CoordinatesSettings.isEnabled() && PhoXiDevice->CoordinatesSettings.CanGet())
    {
        pho::api::PhoXiCoordinatesSettings CoordinatesSettings = PhoXiDevice->CoordinatesSettings;
        if (!PhoXiDevice->CoordinatesSettings.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->CoordinatesSettings.GetLastErrorMessage().c_str());
        }
        PrintCoordinatesSettings(CoordinatesSettings);
    }

    //PhoXiCalibrationSettings
    if (PhoXiDevice->CalibrationSettings.isEnabled() && PhoXiDevice->CalibrationSettings.CanGet())
    {
        pho::api::PhoXiCalibrationSettings CalibrationSettings = PhoXiDevice->CalibrationSettings;
        if (!PhoXiDevice->CalibrationSettings.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->CalibrationSettings.GetLastErrorMessage().c_str());
        }
        PrintCalibrationSettings(CalibrationSettings);
    }

    //FrameOutputSettings
    if (PhoXiDevice->OutputSettings.isEnabled() && PhoXiDevice->OutputSettings.CanGet())
    {
        pho::api::FrameOutputSettings OutputSettings = PhoXiDevice->OutputSettings;
        if (!PhoXiDevice->OutputSettings.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->OutputSettings.GetLastErrorMessage().c_str());
        }
        std::cout << "  OutputSettings: " << std::endl;
        std::cout << "    SendConfidenceMap: " << (OutputSettings.SendConfidenceMap ? "Yes" : "No") << std::endl;
        std::cout << "    SendDepthMap: "      << (OutputSettings.SendDepthMap      ? "Yes" : "No") << std::endl;
        std::cout << "    SendNormalMap: "     << (OutputSettings.SendNormalMap     ? "Yes" : "No") << std::endl;
        std::cout << "    SendPointCloud: "    << (OutputSettings.SendPointCloud    ? "Yes" : "No") << std::endl;
        std::cout << "    SendTexture: "       << (OutputSettings.SendTexture       ? "Yes" : "No") << std::endl;
    }
}

void FullAPIExample::FreerunExample()
{
    //Check if the device is connected
    if (!PhoXiDevice || !PhoXiDevice->isConnected())
    {
        std::cout << "Device is not created, or not connected!" << std::endl;
        return;
    }
    //If it is not in Freerun mode, we need to switch the modes
    if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Freerun)
    {
        std::cout << "Device is not in Freerun mode" << std::endl;
        if (PhoXiDevice->isAcquiring())
        {
            std::cout << "Stopping acquisition" << std::endl;
            //If the device is in Acquisition mode, we need to stop the acquisition
            if (!PhoXiDevice->StopAcquisition())
            {
                throw std::runtime_error("Error in StopAcquistion");
            }
        }
        std::cout << "Switching to Freerun mode " << std::endl;
        //Switching the mode is as easy as assigning of a value, it will call the appropriate calls in the background
        PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Freerun;
        //Just check if did everything run smoothly
        if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
        }
    }

    //Start the device acquisition, if necessary
    if (!PhoXiDevice->isAcquiring())
    {
        if (!PhoXiDevice->StartAcquisition())
        {
            throw std::runtime_error("Error in StartAcquisition");
        }
    }

    //We can clear the current Acquisition buffer -- This will not clear Frames that arrives to the PC after the Clear command is performed
    int ClearedFrames = PhoXiDevice->ClearBuffer();
    std::cout << ClearedFrames << " were cleared from the cyclic buffer" << std::endl;

    //While we checked the state of the StartAcquisition call, this check is not necessary, but it is a good practice
    if (!PhoXiDevice->isAcquiring())
    {
        std::cout << "Device is not acquiring" << std::endl;
        return;
    }

    for (std::size_t i = 0; i < 5; ++i)
    {
        std::cout << "Waiting for frame " << i << std::endl;
        //Get the frame
        pho::api::PFrame Frame = PhoXiDevice->GetFrame(/*You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);
        if (Frame)
        {
            PrintFrameInfo(Frame);
            PrintFrameData(Frame);
        }
        else
        {
            std::cout << "Failed to retrieve the frame!";
        }
    }
}

void FullAPIExample::SoftwareTriggerExample()
{
    //Check if the device is connected
    if (!PhoXiDevice || !PhoXiDevice->isConnected())
    {
        std::cout << "Device is not created, or not connected!" << std::endl;
        return;
    }
    //If it is not in Software trigger mode, we need to switch the modes
    if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software)
    {
        std::cout << "Device is not in Software trigger mode" << std::endl;
        if (PhoXiDevice->isAcquiring())
        {
            std::cout << "Stopping acquisition" << std::endl;
            //If the device is in Acquisition mode, we need to stop the acquisition
            if (!PhoXiDevice->StopAcquisition())
            {
                throw std::runtime_error("Error in StopAcquistion");
            }
        }
        std::cout << "Switching to Software trigger mode " << std::endl;
        //Switching the mode is as easy as assigning of a value, it will call the appropriate calls in the background
        PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
        //Just check if did everything run smoothly
        if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
        }
    }

    //Start the device acquisition, if necessary
    if (!PhoXiDevice->isAcquiring())
    {
        if (!PhoXiDevice->StartAcquisition())
        {
            throw std::runtime_error("Error in StartAcquisition");
        }
    }
    //We can clear the current Acquisition buffer -- This will not clear Frames that arrives to the PC after the Clear command is performed
    int ClearedFrames = PhoXiDevice->ClearBuffer();
    std::cout << ClearedFrames << " frames were cleared from the cyclic buffer" << std::endl;

    //While we checked the state of the StartAcquisition call, this check is not necessary, but it is a good practice
    if (!PhoXiDevice->isAcquiring())
    {
        std::cout << "Device is not acquiring" << std::endl;
        return;
    }
    for (std::size_t i = 0; i < 5; ++i)
    {
        std::cout << "Triggering the " << i << "-th frame" << std::endl;
        int FrameID = PhoXiDevice->TriggerFrame(/*If false is passed here, the device will reject the frame if it is not ready to be triggered, if true us supplied, it will wait for the trigger*/);
        if (FrameID < 0)
        {
            //If negative number is returned trigger was unsuccessful
            std::cout << "Trigger was unsuccessful!" << std::endl;
            continue;
        }
        else
        {
            std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
        }

        std::cout << "Waiting for frame " << i << std::endl;
        //Wait for a frame with specific FrameID. There is a possibility, that frame triggered before the trigger will arrive after the trigger call, and will be retrieved before requested frame
        //  Because of this, the TriggerFrame call returns the requested frame ID, so it can than be retrieved from the Frame structure. This call is doing that internally in background
        pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID/*, You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);
        if (Frame)
        {
            PrintFrameInfo(Frame);
            PrintFrameData(Frame); 
        }
        else
        {
            std::cout << "Failed to retrieve the frame!";
        }
    }
}

void FullAPIExample::SoftwareTriggerAsyncGrabExample()
{
    std::atomic<uint64_t> AsyncFrameID;

    //This callback will be called when new frame is arrived
    auto AsyncGetFrameCallback = [&AsyncFrameID, this](pho::api::PFrame Frame) {
        if (Frame) {
            PrintFrameInfo(Frame);
            PrintFrameData(Frame);
            AsyncFrameID = Frame->Info.FrameIndex;
        }
        else
        {
            std::cout << "Failed to retrieve the frame!" << std::endl;
        }
    };

    //Check if the device is connected
    if (!PhoXiDevice || !PhoXiDevice->isConnected())
    {
        std::cout << "Device is not created, or not connected!" << std::endl;
        return;
    }
    //If it is not in Software trigger mode, we need to switch the modes
    if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software)
    {
        std::cout << "Device is not in Software trigger mode" << std::endl;
        if (PhoXiDevice->isAcquiring())
        {
            std::cout << "Stopping acquisition" << std::endl;
            //If the device is in Acquisition mode, we need to stop the acquisition
            if (!PhoXiDevice->StopAcquisition())
            {
                throw std::runtime_error("Error in StopAcquistion");
            }
        }
        std::cout << "Switching to Software trigger mode " << std::endl;
        //Switching the mode is as easy as assigning of a value, it will call the appropriate calls in the background
        PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
        //Just check if did everything run smoothly
        if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
        }
    }

    //Start the device acquisition, if necessary
    if (!PhoXiDevice->isAcquiring())
    {
        if (!PhoXiDevice->StartAcquisition())
        {
            throw std::runtime_error("Error in StartAcquisition");
        }
    }
    //We can clear the current Acquisition buffer -- This will not clear Frames that arrives to the PC after the Clear command is performed
    int ClearedFrames = PhoXiDevice->ClearBuffer();
    // Enable exclusive asynchronous frame grabbing mode with user defined notification callback
    PhoXiDevice->EnableAsyncGetFrame(std::move(AsyncGetFrameCallback));

    //While we checked the state of the StartAcquisition call, this check is not necessary, but it is a good practice
    if (!PhoXiDevice->isAcquiring())
    {
        std::cout << "Device is not acquiring" << std::endl;
        return;
    }
    int FrameID = -1;
    for (std::size_t i = 0; i < 5; ++i)
    {
        std::cout << "Triggering the " << i << "-th frame" << std::endl;
        FrameID = PhoXiDevice->TriggerFrame(/*If false is passed here, the device will reject the frame if it is not ready to be triggered, if true us supplied, it will wait for the trigger*/);
        if (FrameID < 0)
        {
            //If negative number is returned trigger was unsuccessful
            std::cout << "Trigger was unsuccessful!" << std::endl;
            continue;
        }
        else
        {
            std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
        }
    }

    //Wait for last frame catched in callback
    while (FrameID != AsyncFrameID) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    //Disable asynchronous frame grabbing and switch back to synchronous mode
    PhoXiDevice->DisableAsyncGetFrame();
}

void FullAPIExample::ChangeSettingsExample()
{
    //Check if the device is connected
    if (!PhoXiDevice || !PhoXiDevice->isConnected())
    {
        std::cout << "Device is not created, or not connected!" << std::endl;
        return;
    }
    //Check if the feature is supported and if it we have required access permissions
    //  These checks are not necessary, these have in mind multiple different devices in the future
    if (!PhoXiDevice->CapturingSettings.isEnabled() 
        || !PhoXiDevice->CapturingSettings.CanSet() || !PhoXiDevice->CapturingSettings.CanGet())
    {
        std::cout << "Settings used in example are not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
        return;
    }
    std::cout << "Settings change example" << std::endl;

    //For purpose of this example, we will change the trigger mode to Software Trigger, it is not necessary for the exhibition of desired functionality
    if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software)
    {
        if (PhoXiDevice->isAcquiring())
        {
            if (!PhoXiDevice->StopAcquisition())
            {
                throw std::runtime_error("Error in StopAcquistion");
            }
        }
        PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
        //Just check if did everything run smoothly
        if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
        }
    }

    //Start the device acquisition, if necessary
    if (!PhoXiDevice->isAcquiring())
    {
        if (!PhoXiDevice->StartAcquisition())
        {
            throw std::runtime_error("Error in StartAcquisition");
        }
    }

    int CurrentShutterMultiplier = PhoXiDevice->CapturingSettings->ShutterMultiplier;

    //To change the setting, just assign a new value
    PhoXiDevice->CapturingSettings->ShutterMultiplier = CurrentShutterMultiplier + 1;

    //You can check if the operation succeed
    if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful())
    {
        throw std::runtime_error(PhoXiDevice->CapturingSettings.GetLastErrorMessage().c_str());
    }

    //Get the current Output configuration
    pho::api::FrameOutputSettings CurrentOutputSettings = PhoXiDevice->OutputSettings;
    pho::api::FrameOutputSettings NewOutputSettings = CurrentOutputSettings;
    NewOutputSettings.SendPointCloud = true;
    NewOutputSettings.SendNormalMap = true;
    NewOutputSettings.SendDepthMap = true;
    NewOutputSettings.SendConfidenceMap = true;
    NewOutputSettings.SendTexture = true;
    //Send all outputs
    PhoXiDevice->OutputSettings = NewOutputSettings;

    //Trigger the frame
    int FrameID = PhoXiDevice->TriggerFrame();
    //Check if the frame was successfully triggered
    if (FrameID < 0)
    {
        throw std::runtime_error("Software trigger failed!");
    }

    //Retrieve the frame
    pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID);
    if (Frame)
    {
        //Save the frame for next example
        SampleFrame = Frame;
    }

    //Change the setting back
    PhoXiDevice->OutputSettings = CurrentOutputSettings;
    PhoXiDevice->CapturingSettings->ShutterMultiplier = CurrentShutterMultiplier;

    if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful())
    {
        throw std::runtime_error(PhoXiDevice->CapturingSettings.GetLastErrorMessage().c_str());
    }

    //Try to change device resolution
    if (PhoXiDevice->SupportedCapturingModes.isEnabled() && PhoXiDevice->SupportedCapturingModes.CanGet()
        && PhoXiDevice->CapturingMode.isEnabled() 
        && PhoXiDevice->CapturingMode.CanSet() && PhoXiDevice->CapturingMode.CanGet())
    {
        //Retrieve current capturing mode
        pho::api::PhoXiCapturingMode CurrentCapturingMode = PhoXiDevice->CapturingMode;
        if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());
        }

        //Get all supported modes
        std::vector <pho::api::PhoXiCapturingMode> SupportedCapturingModes = PhoXiDevice->SupportedCapturingModes;
        if (!PhoXiDevice->SupportedCapturingModes.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->SupportedCapturingModes.GetLastErrorMessage().c_str());
        }

        //Cycle trough all other Supported modes, change the settings and grab a frame
        for (std::size_t i = 0; i < SupportedCapturingModes.size(); ++i)
        {
            if (!(SupportedCapturingModes[i] == CurrentCapturingMode))
            {
                PhoXiDevice->CapturingMode = SupportedCapturingModes[i];
                if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
                {
                    throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());
                }
                //Trigger Frame
                int FrameID = PhoXiDevice->TriggerFrame();
                if (FrameID < 0)
                {
                    throw std::runtime_error("Software trigger failed!");
                }

                Frame = PhoXiDevice->GetSpecificFrame(FrameID);
                if (Frame)
                {
                    std::cout << "Arrived Frame Resolution: "
                        << Frame->GetResolution().Width << " x "
                        << Frame->GetResolution().Height << std::endl;
                }
            }
        }

        //Change the mode back
        PhoXiDevice->CapturingMode = CurrentCapturingMode;
        if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
        {
            throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());
        }

    }
}

void FullAPIExample::DataHandlingExample()
{
    //Check if we have SampleFrame Data
    if (!SampleFrame || SampleFrame->Empty())
    {
        std::cout << "Frame does not exist, or has no content!" << std::endl;
        return;
    }

    //We will count the number of measured points
    if (!SampleFrame->PointCloud.Empty())
    {
        int MeasuredPoints = 0;
        pho::api::Point3_32f ZeroPoint(0.0f, 0.0f, 0.0f);
        for (int y = 0; y < SampleFrame->PointCloud.Size.Height; ++y)
        {
            for (int x = 0; x < SampleFrame->PointCloud.Size.Width; ++x)
            {
                if (SampleFrame->PointCloud[y][x] != ZeroPoint)
                {
                    MeasuredPoints++;
                }
            }
        }
        std::cout << "Your sample PointCloud has " << MeasuredPoints << " measured points." << std::endl;

        float *MyLocalCopy = new float[SampleFrame->PointCloud.GetElementsCount() * 3];

        pho::api::Point3_32f *RawPointer = SampleFrame->PointCloud.GetDataPtr();
        memcpy(MyLocalCopy, RawPointer, SampleFrame->PointCloud.GetDataSize());
        //Data are organized as a matrix of X, Y, Z floats, see the documentation for all other types

        delete[] MyLocalCopy;
        //Data from SampleFrame, or all other frames that are returned by the device are copied from the Cyclic buffer and will remain in the memory until the Frame will go out of scope
        //You can specifically call SampleFrame->PointCloud.Clear() to release some of the data
    }

    //You can store the Frame as a ply structure
    //If you don't specify Output folder the PLY file will be saved where FullAPIExample_CSharp.exe is
    const auto outputFolder = OutputFolder.empty() ? std::string() : OutputFolder + DELIMITER;
    const auto sampleFramePly = outputFolder + "SampleFrame.ply";
    std::cout << "Saving frame as 'SampleFrame.ply'" << std::endl;
    if (SampleFrame->SaveAsPly(sampleFramePly, true, true))
    {
        std::cout << "Saved sample frame as PLY to: " << sampleFramePly << std::endl;
    }
    else
    {
        std::cout << "Could not save sample frame as PLY to " << sampleFramePly << " !" << std::endl;
    }
    //You can save scans to any format, you only need to specify path + file name
    //API will look at extension and save the scan in the correct format
    //You can define which options to save (PointCloud, DepthMap, ...) in PhoXi Control application -> Saving options
    //This method has a an optional 2nd parameter: FrameId
    //Use this option to save other scans than the last one
    //Absolute path is prefered
    //If you don't specify Output folder the file will be saved to %APPDATA%\PhotoneoPhoXiControl\ folder on Windows or ~/.PhotoneoPhoXiControl/ on Linux
    const auto sampleFrameAnyFormat = outputFolder + "OtherSampleFrame.tif";
    if (PhoXiDevice->SaveLastOutput(sampleFrameAnyFormat))
    {
        std::cout << "Saved sample frame to: " << sampleFrameAnyFormat << std::endl;
    }
    else
    {
        std::cout << "Could not save sample frame to: " << sampleFrameAnyFormat << " !" << std::endl;
    }

    //If you want OpenCV support, you need to link appropriate libraries and add OpenCV include directory
    //To add the support, add #define PHOXI_OPENCV_SUPPORT before include of PhoXi include files
#ifdef PHOXI_OPENCV_SUPPORT
    if (!SampleFrame->PointCloud.Empty())
    {
        cv::Mat PointCloudMat;
        if (SampleFrame->PointCloud.ConvertTo(PointCloudMat))
        {
            cv::Point3f MiddlePoint = PointCloudMat.at<cv::Point3f>(PointCloudMat.rows/2, PointCloudMat.cols/2);
            std::cout << "Middle point: " << MiddlePoint.x << "; " << MiddlePoint.y << "; " << MiddlePoint.z;
        }
    }
#endif
    //If you want PCL support, you need to link appropriate libraries and add PCL include directory
    //To add the support, add #define PHOXI_PCL_SUPPORT before include of PhoXi include files
#ifdef PHOXI_PCL_SUPPORT
    //The PCL convert will convert the appropriate data into the pcl PointCloud based on the Point Cloud type
    pcl::PointCloud<pcl::PointXYZRGBNormal> MyPCLCloud;
    SampleFrame->ConvertTo(MyPCLCloud);
#endif
}

void FullAPIExample::CorrectDisconnectExample()
{
    //The whole API is designed on C++ standards, using smart pointers and constructor/destructor logic
    //All resources will be closed automatically, but the device state will not be affected -> it will remain connected in PhoXi Control and if in freerun, it will remain Scanning
    //To Stop the device, just
    PhoXiDevice->StopAcquisition();
    //If you want to disconnect and logout the device from PhoXi Control, so it will then be available for other devices, call
    std::cout << "Do you want to logout the device? (0 if NO / 1 if YES) ";
    bool Entry;
    if (!ReadLine(Entry))
    {
        return;
    }
    PhoXiDevice->Disconnect(Entry);
    //The call PhoXiDevice without Logout will be called automatically by destructor
}

void FullAPIExample::PrintFrameInfo(const pho::api::PFrame &Frame)
{
    const pho::api::FrameInfo &FrameInfo = Frame->Info;
    std::cout << "  Frame params: " << std::endl;
    std::cout << "    Frame Index: " << FrameInfo.FrameIndex << std::endl;
    std::cout << "    Frame Timestamp: " << FrameInfo.FrameTimestamp << " s" << std::endl;
    std::cout << "    Frame Acquisition duration: " << FrameInfo.FrameDuration << " ms" << std::endl;
    std::cout << "    Frame Computation duration: " << FrameInfo.FrameComputationDuration << " ms" << std::endl;
    std::cout << "    Frame Transfer duration: " << FrameInfo.FrameTransferDuration << " ms" << std::endl;
    std::cout << "    Sensor Position: ["
        << FrameInfo.SensorPosition.x << "; "
        << FrameInfo.SensorPosition.y << "; "
        << FrameInfo.SensorPosition.z << "]"
        << std::endl;
    std::cout << "    Total scan count: " << FrameInfo.TotalScanCount << std::endl;
}

void FullAPIExample::PrintFrameData(const pho::api::PFrame &Frame)
{
    if (Frame->Empty())
    {
        std::cout << "Frame is empty.";
        return;
    }
    std::cout << "  Frame data: " << std::endl;
    if (!Frame->PointCloud.Empty())
    {
        std::cout << "    PointCloud:    ("
            << Frame->PointCloud.Size.Width << " x "
            << Frame->PointCloud.Size.Height << ") Type: "
            << Frame->PointCloud.GetElementName()
            << std::endl;
    }
    if (!Frame->NormalMap.Empty())
    {
        std::cout << "    NormalMap:     ("
            << Frame->NormalMap.Size.Width << " x "
            << Frame->NormalMap.Size.Height << ") Type: "
            << Frame->NormalMap.GetElementName()
            << std::endl;
    }
    if (!Frame->DepthMap.Empty())
    {
        std::cout << "    DepthMap:      ("
            << Frame->DepthMap.Size.Width << " x "
            << Frame->DepthMap.Size.Height << ") Type: "
            << Frame->DepthMap.GetElementName()
            << std::endl;
    }
    if (!Frame->ConfidenceMap.Empty())
    {
        std::cout << "    ConfidenceMap: ("
            << Frame->ConfidenceMap.Size.Width << " x "
            << Frame->ConfidenceMap.Size.Height << ") Type: "
            << Frame->ConfidenceMap.GetElementName()
            << std::endl;
    }
    if (!Frame->Texture.Empty())
    {
        std::cout << "    Texture:       ("
            << Frame->Texture.Size.Width << " x "
            << Frame->Texture.Size.Height << ") Type: "
            << Frame->Texture.GetElementName()
            << std::endl;
    }
}

void FullAPIExample::PrintCapturingSettings(const pho::api::PhoXiCapturingSettings &CapturingSettings)
{
    std::cout << "  CapturingSettings: "         << std::endl;
    std::cout << "    ShutterMultiplier: "       << CapturingSettings.ShutterMultiplier << std::endl;
    std::cout << "    ScanMultiplier: "          << CapturingSettings.ScanMultiplier << std::endl;
    std::cout << "    CameraOnlyMode: "          << CapturingSettings.CameraOnlyMode << std::endl;
    std::cout << "    AmbientLightSuppression: " << CapturingSettings.AmbientLightSuppression << std::endl;
    std::cout << "    MaximumFPS: "              << CapturingSettings.MaximumFPS << std::endl;
    std::cout << "    SinglePatternExposure: "   << CapturingSettings.SinglePatternExposure << std::endl;
    std::cout << "    CodingStrategy: "          << std::string(CapturingSettings.CodingStrategy) << std::endl;
    std::cout << "    CodingQuality: "           << std::string(CapturingSettings.CodingQuality) << std::endl;
    std::cout << "    TextureSource: "           << std::string(CapturingSettings.TextureSource) << std::endl;
}

void FullAPIExample::PrintProcessingSettings(const pho::api::PhoXiProcessingSettings &ProcessingSettings)
{
    std::cout << "  ProcessingSettings: " << std::endl;
    std::cout << "    Confidence (MaxInaccuracy): " << ProcessingSettings.Confidence << std::endl;
    std::cout << "    CalibrationVolumeOnly: " << ProcessingSettings.CalibrationVolumeOnly;
    PrintVector("MinCameraSpace(in DataCutting)", ProcessingSettings.ROI3D.CameraSpace.min);
    PrintVector("MaxCameraSpace(in DataCutting)", ProcessingSettings.ROI3D.CameraSpace.max);
    PrintVector("MinPointCloudSpace (in DataCutting)", ProcessingSettings.ROI3D.PointCloudSpace.min);
    PrintVector("MaxPointCloudSpace (in DataCutting)", ProcessingSettings.ROI3D.PointCloudSpace.max);
    std::cout << "    MaxCameraAngle: "          << ProcessingSettings.NormalAngle.MaxCameraAngle << std::endl;
    std::cout << "    MaxProjectionAngle: "      << ProcessingSettings.NormalAngle.MaxProjectorAngle << std::endl;
    std::cout << "    MinHalfwayAngle: "         << ProcessingSettings.NormalAngle.MinHalfwayAngle << std::endl;
    std::cout << "    MaxHalfwayAngle: "         << ProcessingSettings.NormalAngle.MaxHalfwayAngle << std::endl;
    std::cout << "    SurfaceSmoothness: "       << std::string(ProcessingSettings.SurfaceSmoothness) << std::endl;
    std::cout << "    NormalsEstimationRadius: " << ProcessingSettings.NormalsEstimationRadius << std::endl;
}

void FullAPIExample::PrintCoordinatesSettings(const pho::api::PhoXiCoordinatesSettings &CoordinatesSettings)
{
    std::cout << "  CoordinatesSettings: " << std::endl;
    PrintMatrix("CustomRotationMatrix", CoordinatesSettings.CustomTransformation.Rotation);
    PrintVector("CustomTranslationVector", CoordinatesSettings.CustomTransformation.Translation);
    PrintMatrix("RobotRotationMatrix", CoordinatesSettings.CustomTransformation.Rotation);
    PrintVector("RobotTranslationVector", CoordinatesSettings.RobotTransformation.Translation);
    std::cout << "    CoordinateSpace: " << std::string(CoordinatesSettings.CoordinateSpace) << std::endl;
    std::cout << "    RecognizeMarkers: " << CoordinatesSettings.RecognizeMarkers << std::endl;
    std::cout << "    MarkerScale: "
        << CoordinatesSettings.MarkersSettings.MarkerScale.Width << " x "
        << CoordinatesSettings.MarkersSettings.MarkerScale.Height
        << std::endl;
}

void FullAPIExample::PrintCalibrationSettings(const pho::api::PhoXiCalibrationSettings &CalibrationSettings)
{
    std::cout << "  CalibrationSettings: " << std::endl;
    std::cout << "    FocusLength: " << CalibrationSettings.FocusLength << std::endl;
    std::cout << "    PixelSize: "
        << CalibrationSettings.PixelSize.Width << " x "
        << CalibrationSettings.PixelSize.Height
        << std::endl;
    PrintMatrix("CameraMatrix", CalibrationSettings.CameraMatrix);
    std::cout << "    DistortionCoefficients: " << std::endl;
    std::cout << "      Format is the following: " << std::endl;
    std::cout << "      (k1, k2, p1, p2[, k3[, k4, k5, k6[, s1, s2, s3, s4[, tx, ty]]]])" << std::endl;

    std::vector<double> distCoeffs = CalibrationSettings.DistortionCoefficients;
    std::stringstream currentDistCoeffsSS;
    int brackets = 0;
    currentDistCoeffsSS << "(";
    currentDistCoeffsSS << distCoeffs[0];
    for (int i = 1; i < distCoeffs.size(); ++i)
    {
        if (i == 4 || i == 5 || i == 8 || i == 12 || i == 14)
        {
            currentDistCoeffsSS << "[";
            ++brackets;
        }
        currentDistCoeffsSS << ", " << distCoeffs[i];
    }
    for (int j = 0; j < brackets; ++j)
    {
        currentDistCoeffsSS << "]";
    }
    currentDistCoeffsSS << ")";
    std::cout << "      " << currentDistCoeffsSS.str() << std::endl;
}

void FullAPIExample::PrintVector(const std::string &name, const pho::api::Point3_64f &vector)
{
    std::cout << "    " << name << ": ["
        << vector.x << "; "
        << vector.y << "; "
        << vector.z << "]"
        << std::endl;
}

void FullAPIExample::PrintMatrix(const std::string &name, const pho::api::CameraMatrix64f &matrix)
{
    std::cout << "    " << name << ": "
        << std::endl << "      ["
        << matrix[0][0] << ", "
        << matrix[0][1] << ", "
        << matrix[0][2] << "]"

        << std::endl << "      ["
        << matrix[1][0] << ", "
        << matrix[1][1] << ", "
        << matrix[1][2] << "]"

        << std::endl << "      ["
        << matrix[2][0] << ", "
        << matrix[2][1] << ", "
        << matrix[2][2] << "]"
        << std::endl;
}

void FullAPIExample::Run()
{
    try
    {
        GetAvailableDevicesExample();
        ConnectPhoXiDeviceExample();
        BasicDeviceStateExample();
        FreerunExample();
        SoftwareTriggerExample();
        SoftwareTriggerAsyncGrabExample();
        ChangeSettingsExample();
        DataHandlingExample();
        CorrectDisconnectExample();
    }
    catch (std::runtime_error &InternalException)
    {
        std::cout << std::endl << "Exception was thrown: " << InternalException.what() << std::endl;
        if (PhoXiDevice->isConnected())
        {
            PhoXiDevice->Disconnect(true);
        }
    }
}

int main(int argc, char *argv[])
{
    FullAPIExample Example;
    Example.Run();
    return 0;
}

