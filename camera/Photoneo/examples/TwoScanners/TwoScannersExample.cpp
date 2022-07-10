/*
 * Photoneo's API Example - TwoScannersExample.cpp
 * Defines the entry point for the console application.
 * Demonstrates how to connect two PhoXi devices in one thread
 * and how to handle received frames.
 */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

#include "PhoXi.h"

#if defined(_WIN32)
#define LOCAL_CROSS_SLEEP(Millis) Sleep(Millis)
#define DELIMITER "\\"
#elif defined(__linux__) || defined(__APPLE__)
#define LOCAL_CROSS_SLEEP(Millis) usleep(Millis * 1000)
#define DELIMITER "/"
#endif

// number of scanners you want to connect to
const size_t SCANNERS_COUNT = 2;

class TwoScannersExample {
private:
    pho::api::PhoXiFactory Factory;
    std::vector<pho::api::PPhoXi> PhoXiDevices;
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList;

    void GetAvailableDevicesExample();
    bool ConnectScanners();
    pho::api::PPhoXi
    ConnectPhoXiDeviceBySerial(const std::string &HardwareIdentification);
    void SoftwareTriggerExample();
    void CorrectDisconnectExample();
    void ProcessFrames(std::vector<pho::api::PFrame> Frames);

    void PrintFrameInfo(const pho::api::PFrame &Frame);
    void PrintFrameData(const pho::api::PFrame &Frame);

    template <class T> bool ReadLine(T &Output) const {
        std::string Input;
        std::getline(std::cin, Input);
        std::stringstream InputSteam(Input);
        return (InputSteam >> Output) ? true : false;
    }
    bool ReadLine(std::string &Output) const {
        std::getline(std::cin, Output);
        return true;
    }

public:
    TwoScannersExample(){};
    ~TwoScannersExample(){};
    void Run();
};

void TwoScannersExample::GetAvailableDevicesExample() {
    // Wait for the PhoXiControl
    while (!Factory.isPhoXiControlRunning()) {
        LOCAL_CROSS_SLEEP(100);
    }
    std::cout << "PhoXi Control Version: " << Factory.GetPhoXiControlVersion()
              << std::endl;
    std::cout << "PhoXi API Version: " << Factory.GetAPIVersion() << std::endl;

    DeviceList = Factory.GetDeviceList();
    std::cout << "PhoXi Factory found " << DeviceList.size() << " devices."
              << std::endl
              << std::endl;
    pho::api::PhoXiDeviceInformation *DeviceInfo;
    for (std::size_t i = 0; i < DeviceList.size(); ++i) {
        DeviceInfo = &DeviceList[i];
        std::cout << "Device: " << i << std::endl;
        std::cout << "  Name:                    " << DeviceInfo->Name << std::endl;
        std::cout << "  Hardware Identification: " << DeviceInfo->HWIdentification << std::endl;
        std::cout << "  Type:                    " << std::string(DeviceInfo->Type) << std::endl;
        std::cout << "  Firmware version:        " << DeviceInfo->FirmwareVersion << std::endl;
        std::cout << "  Variant:                 " << DeviceInfo->Variant << std::endl;
        std::cout << "  IsFileCamera:            " << (DeviceInfo->IsFileCamera ? "Yes" : "No") << std::endl;
        std::cout << "  Status:                  "
                  << (DeviceInfo->Status.Attached
                              ? "Attached to PhoXi Control. "
                              : "Not Attached to PhoXi Control. ")
                  << (DeviceInfo->Status.Ready ? "Ready to connect"
                                               : "Occupied")
                  << std::endl
                  << std::endl;
    }
}

pho::api::PPhoXi TwoScannersExample::ConnectPhoXiDeviceBySerial(
        const std::string &HardwareIdentification) {
    pho::api::PhoXiTimeout Timeout = pho::api::PhoXiTimeout::ZeroTimeout;
    auto PhoXiDevice =
            Factory.CreateAndConnect(HardwareIdentification, Timeout);
    if (PhoXiDevice) {
        std::cout << "Connection to the device " << HardwareIdentification
                  << " was successful!" << std::endl;
        return PhoXiDevice;
    } else {
        std::cout << "Connection to the device " << HardwareIdentification
                  << " was unsuccessful!" << std::endl;
        return nullptr;
    }
}

bool TwoScannersExample::ConnectScanners() {
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        std::cout
                << std::endl
                << "Please enter the Hardware Identification Number of scanner "
                << i + 1 << " :" << std::endl;
        std::string HardwareIdentification;
        if (ReadLine(HardwareIdentification)) {
            // check if scanner is not already connected
            for (size_t j = 0; j < PhoXiDevices.size(); ++j) {
                if (PhoXiDevices.at(j)->HardwareIdentification ==
                    HardwareIdentification) {
                    std::cout << "Already connected " << std::endl;
                    return false;
                }
            }
            auto Device = ConnectPhoXiDeviceBySerial(HardwareIdentification);
            if (Device) {
                PhoXiDevices.push_back(Device);
            } else {
                return false;
            }

        } else {
            return false; // we can not connect to device
        }
    }

    // check if connected to requested num of devices
    if (PhoXiDevices.size() != SCANNERS_COUNT) {
        return false;
    }
    // check if all devices are connected
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        if (!PhoXiDevices[i]->isConnected()) {
            PhoXiDevices.clear();
            return false;
        }
    }

    // all devices are connected
    return true;
}

void TwoScannersExample::CorrectDisconnectExample() {
    // The whole API is designed on C++ standards, using smart pointers and
    // constructor/destructor logic All resources will be closed automatically,
    // but the device state will not be affected -> it will remain connected in
    // PhoXi Control and if in freerun, it will remain Scanning To Stop the
    // device, just
    for (size_t i = 0; i < PhoXiDevices.size(); ++i) {
        if (PhoXiDevices[i]) {
            PhoXiDevices[i]->StopAcquisition();
        }
    }
    // If you want to disconnect and logout the device from PhoXi Control, so it
    // will then be available for other devices, call
    std::cout << "Do you want to logout the device? (0 if NO / 1 if YES) ";
    bool Entry;
    if (!ReadLine(Entry)) {
        return;
    }
    for (size_t i = 0; i < PhoXiDevices.size(); ++i) {
        if (PhoXiDevices[i]) {
            PhoXiDevices[i]->Disconnect(Entry);
        }
    }

    // The call PhoXiDevice without Logout will be called automatically by
    // destructor
}

void TwoScannersExample::PrintFrameInfo(const pho::api::PFrame &Frame) {
    const pho::api::FrameInfo &FrameInfo = Frame->Info;
    std::cout << "  Frame params: " << std::endl;
    std::cout << "    Frame Index: " << FrameInfo.FrameIndex << std::endl;
    std::cout << "    Frame Timestamp: " << FrameInfo.FrameTimestamp << " s"
              << std::endl;
    std::cout << "    Frame Acquisition duration: " << FrameInfo.FrameDuration
              << " ms" << std::endl;
    std::cout << "    Frame Computation duration: "
              << FrameInfo.FrameComputationDuration << " ms" << std::endl;
    std::cout << "    Frame Transfer duration: "
              << FrameInfo.FrameTransferDuration << " ms" << std::endl;
    std::cout << "    Sensor Position: [" << FrameInfo.SensorPosition.x << "; "
              << FrameInfo.SensorPosition.y << "; "
              << FrameInfo.SensorPosition.z << "]" << std::endl;
    std::cout << "    Total scan count: " << FrameInfo.TotalScanCount << std::endl;
}

void TwoScannersExample::PrintFrameData(const pho::api::PFrame &Frame) {
    if (Frame->Empty()) {
        std::cout << "Frame is empty.";
        return;
    }
    std::cout << "  Frame data: " << std::endl;
    if (!Frame->PointCloud.Empty()) {
        std::cout << "    PointCloud:    (" << Frame->PointCloud.Size.Width
                  << " x " << Frame->PointCloud.Size.Height
                  << ") Type: " << Frame->PointCloud.GetElementName()
                  << std::endl;
    }
    if (!Frame->NormalMap.Empty()) {
        std::cout << "    NormalMap:     (" << Frame->NormalMap.Size.Width
                  << " x " << Frame->NormalMap.Size.Height
                  << ") Type: " << Frame->NormalMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->DepthMap.Empty()) {
        std::cout << "    DepthMap:      (" << Frame->DepthMap.Size.Width
                  << " x " << Frame->DepthMap.Size.Height
                  << ") Type: " << Frame->DepthMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->ConfidenceMap.Empty()) {
        std::cout << "    ConfidenceMap: (" << Frame->ConfidenceMap.Size.Width
                  << " x " << Frame->ConfidenceMap.Size.Height
                  << ") Type: " << Frame->ConfidenceMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->Texture.Empty()) {
        std::cout << "    Texture:       (" << Frame->Texture.Size.Width
                  << " x " << Frame->Texture.Size.Height
                  << ") Type: " << Frame->Texture.GetElementName() << std::endl;
    }
}

void TwoScannersExample::SoftwareTriggerExample() {
    // Check if the device is connected
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        if (!PhoXiDevices[i] || !PhoXiDevices[i]->isConnected()) {
            std::cout << "Device is not created, or not connected!"
                      << std::endl;
            return;
        }
    }
    // If it is not in Software trigger mode, we need to switch the modes
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        if (PhoXiDevices[i]->TriggerMode !=
            pho::api::PhoXiTriggerMode::Software) {
            std::cout << "Device is not in Software trigger mode" << std::endl;
            if (PhoXiDevices[i]->isAcquiring()) {
                std::cout << "Stopping acquisition os scanner: " << i
                          << std::endl;
                // If the device is in Acquisition mode, we need to stop the
                // acquisition
                if (!PhoXiDevices[i]->StopAcquisition()) {
                    throw std::runtime_error("Error in StopAcquistion");
                }
            }
            std::cout << "Switching to Software trigger mode " << std::endl;
            // Switching the mode is as easy as assigning of a value, it will
            // call the appropriate calls in the background
            PhoXiDevices[i]->TriggerMode = pho::api::PhoXiTriggerMode::Software;
            // Just check if did everything run smoothly
            if (!PhoXiDevices[i]->TriggerMode.isLastOperationSuccessful()) {
                throw std::runtime_error(
                        PhoXiDevices[i]
                                ->TriggerMode.GetLastErrorMessage()
                                .c_str());
            }
        }
    }

    // Start the device acquisition, if necessary
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        if (!PhoXiDevices[i]->isAcquiring()) {
            if (!PhoXiDevices[i]->StartAcquisition()) {
                throw std::runtime_error("Error in StartAcquisition");
            }
        }
    }
    // We can clear the current Acquisition buffer -- This will not clear Frames
    // that arrives to the PC after the Clear command is performed
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        int ClearedFrames = PhoXiDevices[i]->ClearBuffer();
        std::cout << ClearedFrames
                  << " frames were cleared from the cyclic buffer" << std::endl;
    }

    // While we checked the state of the StartAcquisition call, this check is
    // not necessary, but it is a good practice
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        if (!PhoXiDevices[i]->isAcquiring()) {
            std::cout << "Device: " << i << " is not acquiring" << std::endl;
            return;
        }
    }

    // we are going to trigger 5 frames pairs ( as SCANERS_COUNT = 2)
    for (std::size_t i = 0; i < 5; ++i) {
        std::vector<pho::api::PFrame> Frames;
        // we trigger one frame on each scanner
        for (size_t j = 0; j < SCANNERS_COUNT; ++j) {
            std::cout << "Triggering the " << i << "-th frame on device: " << j
                      << std::endl;
            int FrameID = PhoXiDevices[j]->TriggerFrame(/*If false is passed here, the device will reject the frame if it is not ready to be triggered, if true us supplied, it will wait for the trigger*/);
            if (FrameID < 0) {
                // If negative number is returned trigger was unsuccessful
                std::cout << "Trigger was unsuccessful!" << std::endl;
                continue;
            } else {
                std::cout << "Frame was triggered, Frame Id: " << FrameID
                          << std::endl;
            }

            std::cout << "Waiting for frame " << i << std::endl;
            // Wait for a frame with specific FrameID. There is a possibility,
            // that frame triggered before the trigger will arrive after the
            // trigger call, and will be retrieved before requested frame
            //  Because of this, the TriggerFrame call returns the requested
            //  frame ID, so it can than be retrieved from the Frame structure.
            //  This call is doing that internally in background
            pho::api::PFrame Frame =
                    PhoXiDevices[j]
                            ->GetFrame(); // GetSpecificFrame(FrameID/*, You can
                                          // specify Timeout here - default is
                                          // the Timeout stored in Timeout
                                          // Feature -> Infinity by default*/);

            if (Frame) {
                Frames.push_back(Frame);
            } else {
                std::cout << "Failed to retrieve the frame!";
                break; // we do not want to process incomplete collection of
                       // frames
            }
        }
        // frames are ready for processing
        ProcessFrames(Frames);
    }
}

void TwoScannersExample::ProcessFrames(std::vector<pho::api::PFrame> Frames) {

    // first we check if all frames are ready
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        if (!Frames[i]) {
            std::cout << "Collection of frames is not complete" << std::endl;
            return;
        }
    }

    // now we can process all frames we need
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        {
            // YOUR CODE HERE
            PrintFrameInfo(Frames[i]);
            PrintFrameData(Frames[i]);
        }
    }
}

void TwoScannersExample::Run() {
    try {
        GetAvailableDevicesExample();
        if (!ConnectScanners()) {
            CorrectDisconnectExample();
            std::cout << "Can not connect to all requested scanners."
                      << std::endl;
            return;
        }
        SoftwareTriggerExample();
        CorrectDisconnectExample();
    } catch (std::runtime_error &InternalException) {
        std::cout << std::endl
                  << "Exception was thrown: " << InternalException.what()
                  << std::endl;
        for (auto Device : PhoXiDevices) {
            if (Device->isConnected()) {
                Device->Disconnect(true);
            }
        }
    }
}

int main(int argc, char *argv[]) {
    TwoScannersExample Example;
    Example.Run();
    return 0;
}
