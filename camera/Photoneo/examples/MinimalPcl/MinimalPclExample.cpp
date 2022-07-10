/*
 * Photoneo's API Example - MinimalPclExample.cpp
 * Defines the entry point for the console application.
 * Demonstrates the basic functionality of PhoXi device. This Example shows
 * how to convert Frame to PCL format
 */

#include <iostream>
#include <string>
#include <vector>
#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#define PHOXI_PCL_SUPPORT

#include "PhoXi.h"

// Run software trigger example
void startSoftwareTriggerExample(pho::api::PPhoXi &PhoXiDevice);
// Print out list of device info to standard output
void printDeviceInfoList(
        const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList);
// Print out device info to standard output
void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo);
// Print out frame info to standard output
void printFrameInfo(const pho::api::PFrame &Frame);
// Print out frame data to standard output
void printFrameData(const pho::api::PFrame &Frame);

int main(int argc, char *argv[]) {
    pho::api::PhoXiFactory Factory;

    // Check if the PhoXi Control Software is running
    if (!Factory.isPhoXiControlRunning()) {
        std::cout << "PhoXi Control Software is not running" << std::endl;
        return 0;
    }

    // Get List of available devices on the network
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList =
            Factory.GetDeviceList();
    if (DeviceList.empty()) {
        std::cout << "PhoXi Factory has found 0 devices" << std::endl;
        return 0;
    }
    printDeviceInfoList(DeviceList);

    // Try to connect device opened in PhoXi Control, if any
    pho::api::PPhoXi PhoXiDevice = Factory.CreateAndConnectFirstAttached();
    if (PhoXiDevice) {
        std::cout << "You have already PhoXi device opened in PhoXi Control, "
                     "the API Example is connected to device: "
                  << (std::string)PhoXiDevice->HardwareIdentification
                  << std::endl;
    } else {
        std::cout
                << "You have no PhoXi device opened in PhoXi Control, the API ";
        for (size_t i = 0; i < DeviceList.size(); i++) {
            std::cout << "Example will try to connect to ..."
                      << DeviceList.at(i).HWIdentification << std::endl;
            // wait 5 second for scanner became ready
            PhoXiDevice = Factory.CreateAndConnect(
                    DeviceList.at(i).HWIdentification, 5000);
            if (PhoXiDevice) {
                std::cout << "succesfully connected" << std::endl;
                break;
            }
            if (i == DeviceList.size() - 1) {
                std::cout << "Can not connect to any device" << std::endl;
            }
        }
    }

    // Check if device was created
    if (!PhoXiDevice) {
        std::cout << "Your device was not created!" << std::endl;
        return 0;
    }

    // Check if device is connected
    if (!PhoXiDevice->isConnected()) {
        std::cout << "Your device is not connected" << std::endl;
        return 0;
    }

    // Launch software trigger example
    startSoftwareTriggerExample(PhoXiDevice);

    // Disconnect PhoXi device
    PhoXiDevice->Disconnect();
    return 0;
}

void convertToPCL(const pho::api::PFrame &Frame) {
    std::cout << "Frame " << Frame << "\n";
    pho::api::PhoXiTimeout timeout;
    pcl::PointCloud<pcl::PointXYZ>::Ptr PCLCloud(
            new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZRGBNormal> MyPCLCloud;
    Frame->ConvertTo(MyPCLCloud);

    std::cout << "Number of points in PCL Cloud : " << MyPCLCloud.points.size()
              << std::endl;
}

void startSoftwareTriggerExample(pho::api::PPhoXi &PhoXiDevice) {
    if (PhoXiDevice->isAcquiring()) {
        // Stop acquisition to change trigger mode
        PhoXiDevice->StopAcquisition();
    }

    PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
    std::cout << "Software trigger mode was set" << std::endl;
    PhoXiDevice->ClearBuffer();
    PhoXiDevice->StartAcquisition();
    if (!PhoXiDevice->isAcquiring()) {
        std::cout << "Your device could not start acquisition!" << std::endl;
        return;
    }

    for (int i = 0; i < 5; ++i) {
        std::cout << "Triggering the " << i << "-th frame" << std::endl;
        int FrameID = PhoXiDevice->TriggerFrame();
        if (FrameID < 0) {
            // If negative number is returned trigger was unsuccessful
            std::cout << "Trigger was unsuccessful!" << std::endl;
            continue;
        } else {
            std::cout << "Frame was triggered, Frame Id: " << FrameID
                      << std::endl;
        }

        std::cout << "Waiting for frame " << i << std::endl;
        pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(
                FrameID, pho::api::PhoXiTimeout::Infinity);

        if (Frame) {
            printFrameInfo(Frame);
            printFrameData(Frame);
            convertToPCL(Frame);
        } else {
            std::cout << "Failed to retrieve the frame!" << std::endl;
        }
    }
    PhoXiDevice->StopAcquisition();
}

void printDeviceInfoList(
        const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList) {
    for (std::size_t i = 0; i < DeviceList.size(); ++i) {
        std::cout << "Device: " << i << std::endl;
        printDeviceInfo(DeviceList[i]);
    }
}

void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo) {
    std::cout << "  Name:                    " << DeviceInfo.Name << std::endl;
    std::cout << "  Hardware Identification: " << DeviceInfo.HWIdentification << std::endl;
    std::cout << "  Type:                    " << std::string(DeviceInfo.Type) << std::endl;
    std::cout << "  Firmware version:        " << DeviceInfo.FirmwareVersion << std::endl;
    std::cout << "  Variant:                 " << DeviceInfo.Variant << std::endl;
    std::cout << "  IsFileCamera:            " << (DeviceInfo.IsFileCamera ? "Yes" : "No") << std::endl;
    std::cout << "  Status:                  "
              << (DeviceInfo.Status.Attached
                          ? "Attached to PhoXi Control. "
                          : "Not Attached to PhoXi Control. ")
              << (DeviceInfo.Status.Ready ? "Ready to connect" : "Occupied")
              << std::endl
              << std::endl;
}

void printFrameInfo(const pho::api::PFrame &Frame) {
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

void printFrameData(const pho::api::PFrame &Frame) {
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
