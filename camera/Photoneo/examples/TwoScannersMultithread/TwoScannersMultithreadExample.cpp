/*
 * Photoneo's API Example - TwoScannersMultithreadExample.cpp
 * Defines the entry point for the console application.
 * Demonstrates how to connect two PhoXi devices using two threads and how to
 * handle received frames. If You are looking for simple way how to connect two
 * scanners in one thread please refer to TwoScannersExample.
 */

#include <condition_variable>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
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

class TwoScannersMultithreadExample {
private:
    pho::api::PhoXiFactory Factory;
    std::vector<pho::api::PPhoXi> PhoXiDevices;
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList;
    std::vector<pho::api::PFrame> AcquiringFrames;
    std::vector<pho::api::PFrame> ProcessingFrames;
    std::mutex AcquireMutex;
    std::mutex ProcessMutex;
    std::condition_variable
            AcquireConditionVariable; // used for communication between scanners
                                      // and control thread
    std::string AcquireScanner; // used for communication between scanners and
                                // control thread - contains identification
                                // number of scanner which will acquire frame
    std::condition_variable
            ProcessConditionVariable; // used for communication with frames
                                      // processing thread
    bool Stopping = false;            // used to notify threads to stop
    bool ReadyAcquiring =
            false; // control set this variable to true, when scanner can start
                   // acquiring, scanner thread set this variable to false when
                   // acquiring is done (next scanner can not continue,until
                   // control thread set it again to true)
    bool ReadyProcessing =
            false; // control thread set this variable to true, when data for
                   // processing thread are ready, processing thread set this
                   // variable to false when all frames are processed

    void GetAvailableDevicesExample();
    bool ConnectScanners();
    pho::api::PPhoXi
    ConnectPhoXiDeviceBySerial(const std::string &HardwareIdentification);
    void AcquiringThread(pho::api::PPhoXi Device);
    void CorrectDisconnectExample();
    void ProcessFrames(std::vector<pho::api::PFrame> &Frames);
    void ControlThread();
    void ProcessingThread();
    void StartMultithreadScanning();

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
    TwoScannersMultithreadExample(){};
    ~TwoScannersMultithreadExample(){};
    void Run();
};

void TwoScannersMultithreadExample::GetAvailableDevicesExample() {
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

pho::api::PPhoXi TwoScannersMultithreadExample::ConnectPhoXiDeviceBySerial(
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

bool TwoScannersMultithreadExample::ConnectScanners() {
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

    // finally all devices are connected
    return true;
}

void TwoScannersMultithreadExample::CorrectDisconnectExample() {
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

void TwoScannersMultithreadExample::PrintFrameInfo(
        const pho::api::PFrame &Frame) {
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

void TwoScannersMultithreadExample::PrintFrameData(
        const pho::api::PFrame &Frame) {
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

void TwoScannersMultithreadExample::StartMultithreadScanning() {
    // this thread will notify all other threads what to do
    std::thread ControlThread(
            &TwoScannersMultithreadExample::ControlThread, this);
    // this thread will process acquired frames
    std::thread ProcessingThread(
            &TwoScannersMultithreadExample::ProcessingThread, this);
    // these threads will trigger scans
    std::vector<std::thread> ScannerThreads;
    for (int i = 0; i < SCANNERS_COUNT; ++i) {
        ScannerThreads.emplace_back(std::thread(
                &TwoScannersMultithreadExample::AcquiringThread,
                this,
                PhoXiDevices[i]));
    }
    // we wait until all scanners will finish their jobs
    for (int i = 0; i < SCANNERS_COUNT; ++i) {
        ScannerThreads[i].join();
    }
    // than we can wait for processing thread
    ProcessingThread.join();
    // the last must be control thread as only this one can stop all the others
    ControlThread.join();
}

void TwoScannersMultithreadExample::ControlThread() {
    // trigger 5 scans in this example
    for (int i = 0; i < 5; ++i) {
        // this vector holds acquired frames - must be cleared before use
        AcquiringFrames.clear();
        // notify all scanners to acquire frame
        for (int j = 0; j < SCANNERS_COUNT; ++j) {
            {
                std::lock_guard<std::mutex> lk(AcquireMutex);
                // set which scanner should run
                AcquireScanner =
                        PhoXiDevices[j]->HardwareIdentification.GetValue();
                // all settings are ready for scan
                ReadyAcquiring = true;
            }
            // notify all acquiring threads
            AcquireConditionVariable.notify_all();
            // wait for scan
            {
                std::unique_lock<std::mutex> lk(AcquireMutex);
                // ReadyAcquiring will be set to false by acquire thread
                AcquireConditionVariable.wait(
                        lk, [this] { return !ReadyAcquiring; });
            }
        }
        // make a copy of data for processing thread
        {
            std::unique_lock<std::mutex> lk(ProcessMutex);
            ProcessConditionVariable.wait(
                    lk, [this] { return !ReadyProcessing; });
            ProcessingFrames = AcquiringFrames;
            ReadyProcessing = true;
        }
        // notify processing threads
        ProcessConditionVariable.notify_one();
    }
    // wait for processing of the last frames
    {
        std::unique_lock<std::mutex> lk(ProcessMutex);
        ProcessConditionVariable.wait(lk, [this] { return !ReadyProcessing; });
        ProcessingFrames = AcquiringFrames;
        ReadyProcessing = true;
    }

    // we set that our job is done
    Stopping = true;

    // notify all acquire threads to stop
    for (int j = 0; j < SCANNERS_COUNT; ++j) {
        AcquireConditionVariable.notify_all();
    }
    // notify process thread to stop
    ProcessConditionVariable.notify_all();
}

// run this
void TwoScannersMultithreadExample::AcquiringThread(
        pho::api::PPhoXi PhoXiDevice) {
    if (!PhoXiDevice || !PhoXiDevice->isConnected()) {
        std::cout << "Device is not created, or not connected!" << std::endl;
        return;
    }

    // If it is not in Software trigger mode, we need to switch the modes
    if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software) {
        std::cout << "Device is not in Software trigger mode" << std::endl;
        if (PhoXiDevice->isAcquiring()) {
            std::cout << "Stopping acquisition os scanner: "
                      << PhoXiDevice->HardwareIdentification.GetValue()
                      << std::endl;
            // If the device is in Acquisition mode, we need to stop the
            // acquisition
            if (!PhoXiDevice->StopAcquisition()) {
                throw std::runtime_error("Error in StopAcquistion");
            }
        }
        std::cout << "Switching to Software trigger mode " << std::endl;
        // Switching the mode is as easy as assigning of a value, it will call
        // the appropriate calls in the background
        PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
        // Just check if everything run smoothly
        if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful()) {
            throw std::runtime_error(
                    PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
        }
    }

    // Start the device acquisition, if necessary
    if (!PhoXiDevice->isAcquiring()) {
        if (!PhoXiDevice->StartAcquisition()) {
            throw std::runtime_error("Error in StartAcquisition");
        }
    }
    // We can clear the current Acquisition buffer -- This will not clear Frames
    // that arrives to the PC after the Clear command is performed
    int ClearedFrames = PhoXiDevice->ClearBuffer();
    std::cout << ClearedFrames << " frames were cleared from the cyclic buffer"
              << std::endl;

    // While we checked the state of the StartAcquisition call, this check is
    // not necessary, but it is a good practice
    if (!PhoXiDevice->isAcquiring()) {
        std::cout << "Device: "
                  << PhoXiDevice->HardwareIdentification.GetValue()
                  << " is not acquiring" << std::endl;
        return;
    }

    // thread remain in this loop until we want it to stop (set Stopping = true)
    while (!Stopping) {
        {
            std::unique_lock<std::mutex> lk(AcquireMutex);
            // continue only when stopping or we should acquire a frame
            AcquireConditionVariable.wait(lk, [this, &PhoXiDevice] {
                return Stopping ||
                       (ReadyAcquiring &&
                        (AcquireScanner ==
                         PhoXiDevice->HardwareIdentification.GetValue()));
            });
            if (Stopping) {
                break;
            }

            std::cout << "Triggering the frame on device: "
                      << PhoXiDevice->HardwareIdentification.GetValue()
                      << std::endl;
            int FrameID = PhoXiDevice->TriggerFrame(/*If false is passed here, the device will reject the frame if it is not ready to be triggered, if true us supplied, it will wait for the trigger*/);
            if (FrameID < 0) {
                // If negative number is returned trigger was unsuccessful
                std::cout << "Trigger was unsuccessful!" << std::endl;
            } else {
                std::cout << "Frame was triggered, Frame Id: " << FrameID
                          << std::endl;
            }

            std::cout << "Waiting for frame " << std::endl;
            // Wait for a frame with specific FrameID. There is a possibility,
            // that frame triggered before the trigger will arrive after the
            // trigger call, and will be retrieved before requested frame
            //  Because of this, the TriggerFrame call returns the requested
            //  frame ID, so it can than be retrieved from the Frame structure.
            //  This call is doing that internally in background
            pho::api::PFrame Frame =
                    PhoXiDevice
                            ->GetFrame(); // GetSpecificFrame(FrameID/*, You
                                          // can specify Timeout here - default
                                          // is the Timeout stored in Timeout
                                          // Feature -> Infinity by default*/);

            if (Frame) {
                // add frame to buffer
                AcquiringFrames.push_back(Frame);
            } else {
                std::cout << "Failed to retrieve the frame!";
                break; // we do not want to process incomplete collection of
                       // frames
            }
            ReadyAcquiring = false;
        }
        // notify control thread
        AcquireConditionVariable.notify_all();
    }
}

void TwoScannersMultithreadExample::ProcessingThread() {
    while (!Stopping) {
        {
            // continue only when stopping or should process a frame
            std::unique_lock<std::mutex> lk(AcquireMutex);
            ProcessConditionVariable.wait(
                    lk, [this] { return Stopping || ReadyProcessing; });
            if (Stopping) {
                ReadyProcessing = false;
                break;
            }
            ProcessFrames(ProcessingFrames);
            ReadyProcessing = false;
        }
        // notiffy control thread
        ProcessConditionVariable.notify_one();
    }
}

void TwoScannersMultithreadExample::ProcessFrames(
        std::vector<pho::api::PFrame> &Frames) {
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

void TwoScannersMultithreadExample::Run() {
    try {
        GetAvailableDevicesExample();
        if (!ConnectScanners()) {
            CorrectDisconnectExample();
            std::cout << "Can not connect all requested scanners." << std::endl;
            return;
        }
        StartMultithreadScanning();
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
    TwoScannersMultithreadExample Example;
    Example.Run();
    return 0;
}
