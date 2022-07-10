/*
 * Photoneo's API Example - ReadPointCloudExample.cpp
 * Defines the entry point for the console application.
 * Demonstrates how to get work with Point cloud of
 * received frames.
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

class ReadPointCloudExample {
private:
    pho::api::PhoXiFactory Factory;
    pho::api::PPhoXi PhoXiDevice;
    pho::api::PFrame SampleFrame;
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList;
    std::string OutputFolder = "";

    void GetAvailableDevicesExample();
    void ConnectPhoXiDeviceBySerialExample();
    void SoftwareTriggerExample();
    void DataHandlingExample();
    void CorrectDisconnectExample();

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
    ReadPointCloudExample(){};
    ~ReadPointCloudExample(){};
    void Run();
};

void ReadPointCloudExample::GetAvailableDevicesExample() {
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

void ReadPointCloudExample::ConnectPhoXiDeviceBySerialExample() {
    std::cout << std::endl
              << "Please enter the Hardware Identification Number: ";
    std::string HardwareIdentification;
    if (!ReadLine(HardwareIdentification)) {
        std::cout << "Incorrect input!" << std::endl;
        return;
    }

    pho::api::PhoXiTimeout Timeout = pho::api::PhoXiTimeout::ZeroTimeout;
    PhoXiDevice = Factory.CreateAndConnect(HardwareIdentification, Timeout);
    if (PhoXiDevice) {
        std::cout << "Connection to the device " << HardwareIdentification
                  << " was Successful!" << std::endl;
    } else {
        std::cout << "Connection to the device " << HardwareIdentification
                  << " was Unsuccessful!" << std::endl;
    }
}

void ReadPointCloudExample::CorrectDisconnectExample() {
    if (PhoXiDevice) {
        // The whole API is designed on C++ standards, using smart pointers and
        // constructor/destructor logic All resources will be closed automatically,
        // but the device state will not be affected -> it will remain connected in
        // PhoXi Control and if in freerun, it will remain Scanning. To Stop the
        // acquisition, just call
        PhoXiDevice->StopAcquisition();
        // If you want to disconnect and logout the device from PhoXi Control, so it
        // will then be available for other devices, call
        std::cout << "Do you want to logout the device? (0 if NO / 1 if YES) ";
        bool wantsToLogOut = true;
        if (!ReadLine(wantsToLogOut)) {
            wantsToLogOut = true;
        }
        PhoXiDevice->Disconnect(wantsToLogOut);
        // The call PhoXiDevice without Logout will be called automatically by
        // destructor
    }
}

void ReadPointCloudExample::PrintFrameInfo(const pho::api::PFrame &Frame) {
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

void ReadPointCloudExample::PrintFrameData(const pho::api::PFrame &Frame) {
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

void ReadPointCloudExample::SoftwareTriggerExample() {
    // Check if the device is connected
    if (!PhoXiDevice || !PhoXiDevice->isConnected()) {
        std::cout << "Device is not created, or not connected!" << std::endl;
        return;
    }
    // If it is not in Software trigger mode, we need to switch the modes
    if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software) {
        std::cout << "Device is not in Software trigger mode" << std::endl;
        if (PhoXiDevice->isAcquiring()) {
            std::cout << "Stopping acquisition" << std::endl;
            // If the device is in Acquisition mode, we need to stop the
            // acquisition
            if (!PhoXiDevice->StopAcquisition()) {
                throw std::runtime_error("Error in StopAcquisition");
            }
        }
        std::cout << "Switching to Software trigger mode " << std::endl;
        // Switching the mode is as easy as assigning of a value, it will call
        // the appropriate calls in the background
        PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
        // Just check if did everything run smoothly
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
        std::cout << "Device is not acquiring" << std::endl;
        return;
    }
    for (std::size_t i = 0; i < 5; ++i) {
        std::cout << "Triggering the " << i << "-th frame" << std::endl;
        int FrameID = PhoXiDevice->TriggerFrame(/*If false is passed here, the device will reject the frame if it is not ready to be triggered, if true us supplied, it will wait for the trigger*/);
        if (FrameID < 0) {
            // If negative number is returned trigger was unsuccessful
            std::cout << "Trigger was unsuccessful!" << std::endl;
            continue;
        } else {
            std::cout << "Frame was triggered, Frame Id: " << FrameID
                      << std::endl;
        }

        std::cout << "Waiting for frame " << i << std::endl;
        // Wait for a frame with specific FrameID. There is a possibility, that
        // frame triggered before the trigger will arrive after the trigger
        // call, and will be retrieved before requested frame
        //  Because of this, the TriggerFrame call returns the requested frame
        //  ID, so it can than be retrieved from the Frame structure. This call
        //  is doing that internally in background
        pho::
                api::
                        PFrame Frame = PhoXiDevice->GetSpecificFrame(FrameID /*, You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);
        if (Frame) {
            PrintFrameInfo(Frame);
            PrintFrameData(Frame);
        } else {
            std::cout << "Failed to retrieve the frame!";
        }
        SampleFrame = Frame;
        DataHandlingExample();
    }
}

void ReadPointCloudExample::DataHandlingExample() {
    // Check if we have SampleFrame Data
    if (!SampleFrame || SampleFrame->Empty()) {
        std::cout << "Frame does not exist, or has no content!" << std::endl;
        return;
    }

    // We will count the number of measured points
    if (!SampleFrame->PointCloud.Empty()) {
        int MeasuredPoints = 0;
        pho::api::Point3_32f ZeroPoint(0.0f, 0.0f, 0.0f);
        for (int y = 0; y < SampleFrame->PointCloud.Size.Height; ++y) {
            for (int x = 0; x < SampleFrame->PointCloud.Size.Width; ++x) {
                if (SampleFrame->PointCloud[y][x] != ZeroPoint) {
                    MeasuredPoints++;
                }
            }
        }
        std::cout << "Your sample PointCloud has " << MeasuredPoints
                  << " measured points." << std::endl;

        float *MyLocalCopy =
                new float[SampleFrame->PointCloud.GetElementsCount() * 3];

        pho::api::Point3_32f *RawPointer = SampleFrame->PointCloud.GetDataPtr();
        memcpy(MyLocalCopy, RawPointer, SampleFrame->PointCloud.GetDataSize());
        // Data are organized as a matrix of X, Y, Z floats, see the
        // documentation for all other types

        delete[] MyLocalCopy;
        // Data from SampleFrame, or all other frames that are returned by the
        // device are copied from the Cyclic buffer and will remain in the
        // memory until the Frame will go out of scope You can specifically call
        // SampleFrame->PointCloud.Clear() to release some of the data
    }

    // You can store the Frame as a ply structure
    // If you don't specify Output folder the PLY file will be saved where
    // FullAPIExample_CSharp.exe is
    const auto outputFolder =
            OutputFolder.empty() ? std::string() : OutputFolder + DELIMITER;
    const auto sampleFramePly = outputFolder + "SampleFrame.ply";
    std::cout << "Saving frame as 'SampleFrame.ply'" << std::endl;
    if (SampleFrame->SaveAsPly(sampleFramePly, true, true)) {
        std::cout << "Saved sample frame as PLY to: " << sampleFramePly
                  << std::endl;
    } else {
        std::cout << "Could not save sample frame as PLY to " << sampleFramePly
                  << " !" << std::endl;
    }
    // You can save scans to any format, you only need to specify path + file
    // name API will look at extension and save the scan in the correct format
    // You can define which options to save (PointCloud, DepthMap, ...) in PhoXi
    // Control application -> Saving options This method has a an optional 2nd
    // parameter: FrameId Use this option to save other scans than the last one
    // Absolute path is prefered
    // If you don't specify Output folder the file will be saved to
    // %APPDATA%\PhotoneoPhoXiControl\ folder on Windows or
    // ~/.PhotoneoPhoXiControl/ on Linux
    const auto sampleFrameAnyFormat = outputFolder + "OtherSampleFrame.tif";
    if (PhoXiDevice->SaveLastOutput(sampleFrameAnyFormat)) {
        std::cout << "Saved sample frame to: " << sampleFrameAnyFormat
                  << std::endl;
    } else {
        std::cout << "Could not save sample frame to: " << sampleFrameAnyFormat
                  << " !" << std::endl;
    }

    // If you want OpenCV support, you need to link appropriate libraries and
    // add OpenCV include directory To add the support, add #define
    // PHOXI_OPENCV_SUPPORT before include of PhoXi include files For details
    // check also MinimalOpenCVExample
#ifdef PHOXI_OPENCV_SUPPORT
    if (!SampleFrame->PointCloud.Empty()) {
        cv::Mat PointCloudMat;
        if (SampleFrame->PointCloud.ConvertTo(PointCloudMat)) {
            cv::Point3f MiddlePoint = PointCloudMat.at<cv::Point3f>(
                    PointCloudMat.rows / 2, PointCloudMat.cols / 2);
            std::cout << "Middle point: " << MiddlePoint.x << "; "
                      << MiddlePoint.y << "; " << MiddlePoint.z;
        }
    }
#endif
    // If you want PCL support, you need to link appropriate libraries and add
    // PCL include directory To add the support, add #define PHOXI_PCL_SUPPORT
    // before include of PhoXi include files For details check also
    // MinimalPclExample
#ifdef PHOXI_PCL_SUPPORT
    // The PCL convert will convert the appropriate data into the pcl PointCloud
    // based on the Point Cloud type
    pcl::PointCloud<pcl::PointXYZRGBNormal> MyPCLCloud;
    SampleFrame->ConvertTo(MyPCLCloud);
#endif
}

void ReadPointCloudExample::Run() {
    try {
        GetAvailableDevicesExample();
        ConnectPhoXiDeviceBySerialExample();
        SoftwareTriggerExample();
        CorrectDisconnectExample();
    } catch (std::runtime_error &InternalException) {
        std::cout << std::endl
                  << "Exception was thrown: " << InternalException.what()
                  << std::endl;
        if (PhoXiDevice->isConnected()) {
            PhoXiDevice->Disconnect(true);
        }
    }
}

int main(int argc, char *argv[]) {
    ReadPointCloudExample Example;
    Example.Run();
    return 0;
}
