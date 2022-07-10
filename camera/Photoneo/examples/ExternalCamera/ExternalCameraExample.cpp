// ExternalExample.cpp : Defines the entry point for the console application.
#include "PhoXi.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#if defined(_WIN32)
#include <windows.h>
#elif defined (__linux__)
#include <unistd.h>
#endif
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "PhoXiOpenCVSupport.h"
#include "PhoXiAdditionalCamera.h"

#if defined(_WIN32)
#define LOCAL_CROSS_SLEEP(Millis) Sleep(Millis)
#elif defined (__linux__) || defined(__APPLE__)
#define LOCAL_CROSS_SLEEP(Millis) usleep(Millis * 1000)
#endif

using namespace pho::api;
using namespace AdditionalCamera;

enum class DepthMapSettingsResult {
    Correct,
    Incorrect,
    FileMissing
};

//The whole api is in namespace pho (Photoneo) :: api
struct CalibrationSettings {
    double FocalLength = 0;
    double PixelSize = 0;
    std::string MarkersPositions = std::string();
    std::vector<Texture32f> Images;
    std::vector<Texture32f> Frames;
};

struct DepthMapSettings {
    AdditionalCameraCalibration Calibration;
    DepthMap32f DepthMap;
    int FromFileCameraCount = 0;
    int FromDeviceCount = 0;
};

class PhoXiExternalCameraExamples {
private:
    template<class T>
    bool ReadLine(T &Output) const {
        std::string Input;
        std::getline(std::cin, Input);
        std::stringstream InputSteam(Input);
        if (InputSteam >> Output) {
            return true;
        }
        return false;
    }
    bool ReadLine(std::string &Output) const {
        std::getline(std::cin, Output);
        return true;
    }
    
    const std::string FileCameraHardwareIdentification = "ExampleFileCamera";
    const std::string FileCameraPrefix = "fileCamera_";
    const std::string DevicePrefix = "device_";
    std::string currentPrefix = "";
    std::string dataFolderPath;
    std::string settingsFolderPath;
    std::string delimiter;
    std::string projectDirectory;
public:
    PhoXiExternalCameraExamples(
        const std::string &executablePath, 
        const std::string &settingsPath = std::string()) {
        try {
            projectDirectory = GetProjectDirectory(executablePath);
            settingsFolderPath = !settingsPath.empty() ? 
                settingsPath :
                projectDirectory + delimiter + "Settings" + delimiter;
            dataFolderPath = projectDirectory + delimiter + "Data" + delimiter;
            RunExample();
        }
        catch (std::runtime_error &InternalException) {
            std::cout << std::endl << "Exception was thrown: " << InternalException.what() << std::endl;
        }
    }

    void RunExample() {
        std::size_t Index;
        while (true) {
            PrintCalibrationOrDepthMapinfo();

            if (!ReadLine(Index)) {
                std::cout << "Incorrect input!" << std::endl;
                continue;
            }

            switch (Index) {
            case 1:
                TestCalibration();
                break;
            case 2:
                TestDepthMap();
                break;
            case 3:
                Disconnect();
                std::cout << "Exiting application..." << std::endl;
                exit(0);
            default:
                continue;
            }
        }
    }

#pragma region Calibration
    void TestCalibration() {
        PrintCalibrationInfo();
        LoadCalibrationSettings();
        PrintCalibrationSettings();
        std::size_t Index;
        while (true) {
            PrintCalibrationFromFileOrConnectToScannerInfo();
            if (!ReadLine(Index)) {
                std::cout << "Incorrect input!" << std::endl;
                continue;
            }

            switch (Index) {
            case 1:
                PrintCalibrationFromFileInfo();
                LoadImagesFromFile();
                LoadFramesFromFile();
                AttachPrawFile();
                break;
            case 2: {
                GetAvailableDevices();
                PrintCalibrationConnectToScannerInfo();
                ConnectPhoXiDeviceByPhoXiDeviceInformationEntry();
                auto shouldContinue = true;
                while (shouldContinue) {
                    std::cout << std::endl; std::cout << std::endl;
                    TriggerScanAndGetTexture(1);
                    GetImageFromExternalCamera();
                    PrintCalibrationShouldContinueInfo();
                    std::size_t InnerIndex;
                    if (!ReadLine(InnerIndex)) {
                        std::cout << "Incorrect input!" << std::endl;
                        continue;
                    }

                    switch (InnerIndex) {
                    case 1:
                        shouldContinue = true;
                        continue;
                    case 2:
                        shouldContinue = false;
                        continue;
                    default: 
                        std::cout << "Incorrect input!" << std::endl;
                    }
                }
                break;
            }
            default:
                std::cout << "Incorrect input!" << std::endl;
                continue;
            }
            break;
        }
        GetCalibration();
        Disconnect();
    }

    void GetCalibration() const {
        std::cout << std::endl << std::endl;
        std::cout << "Starting calibration..." << std::endl;
        AdditionalCameraCalibration outputCalibration;
        Calibrator Calibrator(PhoXiDevice);
        if (Calibrator.Calibrate(
            CalibrationSetting.Frames,
            CalibrationSetting.Images,
            CalibrationSetting.FocalLength,
            CalibrationSetting.PixelSize,
            CalibrationSetting.MarkersPositions,
            outputCalibration)) {
            std::cout << "External camera calibration was successfull!" << std::endl;
            outputCalibration.SaveToFile("calibration.txt");
            std::cout << "Calibration was saved to 'calibration.txt' in project folder." << std::endl;
        }
        else {
            std::cout << "External camera calibration was NOT successfull!" << std::endl;
        }
    }

    void LoadFramesFromFile() {
        std::cout << "Loading frames from files..." << std::endl;
        std::vector<std::string> frameFiles;
        for (size_t i = 0; i < 10; ++i) {
            frameFiles.push_back(dataFolderPath + "frame" + std::to_string(i + 1) + ".png");
        }
        std::cout << "Loaded " << frameFiles.size() << " frames from 'Data' folder." << std::endl;
        for (size_t i = 0; i < 10; ++i) {
            const auto cvImage = cv::imread(frameFiles.at(i), CV_LOAD_IMAGE_GRAYSCALE);
            Texture32f resultFrame;
            ConvertOpenCVMatToMat2D(cvImage, resultFrame);
            CalibrationSetting.Frames.push_back(resultFrame);
        }
    }

    void LoadImagesFromFile() {
        std::cout << "Loading images from files..." << std::endl;
        std::vector<std::string> imageFiles;
        for (size_t i = 0; i < 10; ++i) {
            imageFiles.push_back(dataFolderPath + "image" + std::to_string(i + 1) + ".png");
        }
        std::cout << "Loaded " << imageFiles.size() << " images from 'Data' folder." << std::endl;
        for (size_t i = 0; i < 10; ++i) {
            auto cvImage = cv::imread(imageFiles.at(i), CV_LOAD_IMAGE_GRAYSCALE);
            Texture32f resultImage;
            ConvertOpenCVMatToMat2D(cvImage, resultImage);
            CalibrationSetting.Images.push_back(resultImage);
        }
    }

    void LoadCalibrationSettings() {
        std::ifstream focalLengthFile(settingsFolderPath + "FocalLength.txt");
        if (focalLengthFile.is_open()) {
            auto line = std::string();
            std::getline(focalLengthFile, line);
            CalibrationSetting.FocalLength = std::stod(line);
        }

        std::ifstream pixelSizeFile(settingsFolderPath + "PixelSize.txt");
        if (pixelSizeFile.is_open()) {
            auto line = std::string();
            std::getline(pixelSizeFile, line);
            CalibrationSetting.PixelSize = std::stod(line);
        }

        CalibrationSetting.MarkersPositions = settingsFolderPath + "MarkersPositions.txt";
    }
#pragma endregion 

#pragma region Depth Map
    void TestDepthMap() {
        PrintDepthMapInfo();
        const auto calibrationResult = LoadDepthMapSettings();
        switch (calibrationResult) {
        case DepthMapSettingsResult::Correct:
            PrintDepthMapSettings();
            break;
        case DepthMapSettingsResult::Incorrect:
            PrintDepthMapSettingsIncorrectError();
            exit(0);
        case DepthMapSettingsResult::FileMissing:
            PrintDepthMapSettingsMissingError();
            exit(0);
        default:
            PrintDepthMapSettingsIncorrectError();
            exit(0);
        }
         
        while (true) {
            PrintDepthMapFromFileOrConnectToScannerInfo();
            std::size_t Index;
            if (!ReadLine(Index)) {
                std::cout << "Incorrect input!" << std::endl;
                continue;
            }

            switch (Index) {
            case 1:
                currentPrefix = FileCameraPrefix;
                AttachPrawFile();
                std::cout << std::endl; std::cout << std::endl;
                TriggerScanAndGetTexture(1);
                std::cout << std::endl; std::cout << std::endl;
                GetAlignedDepthMap();
                break;
            case 2: {
                currentPrefix = DevicePrefix;
                GetAvailableDevices();
                PrintDepthMapConnectToScannerInfo();
                ConnectPhoXiDeviceByPhoXiDeviceInformationEntry();
                std::cout << std::endl; std::cout << std::endl;
                auto shouldContinue = true;
                while (shouldContinue) {
                    TriggerScanAndGetTexture(1);
                    GetImageFromExternalCamera();
                    std::cout << std::endl; std::cout << std::endl;
                    GetAlignedDepthMap();
                    PrintDepthMapShouldContinueInfo();
                    std::size_t shouldContinueIndex;
                    if (!ReadLine(shouldContinueIndex)) {
                        std::cout << "Incorrect input!" << std::endl;
                        continue;
                    }

                    switch (shouldContinueIndex) {
                        case 1:
                            shouldContinue = true;
                            break;
                        case 2:
                            shouldContinue = false;
                            break;
                        default:
                            std::cout << "Incorrect input!" << std::endl;
                    }
                }
                break;
            }
            default:
                std::cout << "Incorrect input!" << std::endl;
                continue;
            }
            break;
        }
        Disconnect();
    }

    void GetAlignedDepthMap() {
        Aligner Aligner(PhoXiDevice, DepthMapSetting.Calibration);
        if (Aligner.GetAlignedDepthMap(DepthMapSetting.DepthMap)) {
            cv::Mat ResultDepthMap;
            ConvertMat2DToOpenCVMat(DepthMapSetting.DepthMap, ResultDepthMap);
            const auto index = currentPrefix == FileCameraPrefix 
            ? ++DepthMapSetting.FromFileCameraCount 
            : ++DepthMapSetting.FromDeviceCount;
            imwrite(currentPrefix + std::to_string(index) + ".jpg", ResultDepthMap);
            std::cout << "Aligned depth map was computed successfully!" << std::endl;
            std::cout << "Depth map was saved to '" << currentPrefix << 
                std::to_string(index) << 
                ".jpg' in the project folder." << std::endl;
        }
        else {
            std::cout << "Computation of aligned depth map was NOT successful!" << std::endl;
        }
    }
#pragma endregion 

#pragma region Common
    void AttachPrawFile() {
        std::vector<std::string> prawFiles;
        prawFiles.push_back(dataFolderPath + "1.praw");
        AttachedFileCameraName = Factory.AttachFileCamera(FileCameraHardwareIdentification, prawFiles);
        std::cout << std::endl << std::endl;
        std::cout << "Connecting to the device " << AttachedFileCameraName << "..." << std::endl;
        if (!AttachedFileCameraName.empty()) {
            PhoXiDevice = Factory.CreateAndConnect(AttachedFileCameraName, PhoXiTimeout::Infinity);
            if (PhoXiDevice->Connect()) {
                std::cout << "Connection to the device " << AttachedFileCameraName << " was successfull!";
            }
            else {
                std::cout << "Connection to the device " << AttachedFileCameraName << " was NOT successfull!";
            }
        }
        else {
            std::cout << "Error attaching praw file.";
            exit(-1);
        }
    }

    void GetImageFromExternalCamera() {
        std::cout << "Retrieving image from external camera..." << std::endl;
        // TODO: implement this method to get image from external camera
        // start
        // ...
        // end
        // this is the conversion from cv::Mat to Texture32f
        cv::Mat cvImage;
        Texture32f resultImage;
        ConvertOpenCVMatToMat2D(cvImage, resultImage);
        std::cout << "Image from external camera retrieved." << std::endl;
        CalibrationSetting.Images.push_back(resultImage);
    }

    std::vector<Texture32f> TriggerScanAndGetTexture(const int numberOfScans) {
        std::vector<Texture32f> FrameTextures;
        //Check if the device is connected
        if (PhoXiDevice && PhoXiDevice->isConnected()) {
            //If it is not in Software trigger mode, we need to switch the modes
            if (PhoXiDevice->TriggerMode != PhoXiTriggerMode::Software) {
                std::cout << "Device is not in Software trigger mode" << std::endl;
                if (PhoXiDevice->isAcquiring()) {
                    std::cout << "Stopping acquisition" << std::endl;
                    //If the device is in Acquisition mode, we need to stop the acquisition
                    if (!PhoXiDevice->StopAcquisition()) {
                        throw std::runtime_error("Error in StopAcquistion");
                    }
                }
                std::cout << "Switching to Software trigger mode " << std::endl;
                //Switching the mode is as easy as assigning of a value, it will call the appropriate calls in the background
                PhoXiDevice->TriggerMode = PhoXiTriggerMode::Software;
                //Just check if did everything run smoothly
                if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
            }
            //Start the device acquisition, if necessary
            if (!PhoXiDevice->isAcquiring()) {
                if (!PhoXiDevice->StartAcquisition()) {
                    throw std::runtime_error("Error in StartAcquisition");
                }
            }
            //We can clear the current Acquisition buffer -- This will not clear Frames that arrives to the PC after the Clear command is performed
            auto ClearedFrames = PhoXiDevice->ClearBuffer();

            //While we checked the state of the StartAcquisition call, this check is not necessary, but it is a good practice
            if (PhoXiDevice->isAcquiring()) {
                for (std::size_t i = 0; i < numberOfScans; i++) {
                    std::cout << "Triggering a scan..." << std::endl;
                    const auto FrameID =
                        PhoXiDevice->TriggerFrame(true, true);
                    if (FrameID < 0) {
                        //If negative number is returned trigger was unsuccessful
                        std::cout << "Trigger was unsuccessful!" << std::endl;
                        continue;
                    }
                    std::cout << "Scan was triggered, Frame Id: " << FrameID << std::endl;
                    //Wait for a frame with specific FrameID. There is a possibility, that frame triggered before the trigger will arrive after the trigger call, and will be retrieved before requested frame
                    //  Because of this, the TriggerFrame call returns the requested frame ID, so it can than be retrieved from the Frame structure. This call is doing that internally in background
                    const auto Frame =
                        PhoXiDevice->GetFrame(/*You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);
                    if (Frame) {
                        std::cout << "Frame successfully retrieved." << std::endl;
                        if (!Frame->Empty()) {
                            if (!Frame->Texture.Empty()) {
                                CalibrationSetting.Frames.push_back(Frame->Texture);
                            }
                            else {
                                std::cout << "Frame Texture is empty.";
                            }
                        }
                        else {
                            std::cout << "Frame is empty.";
                        }
                    }
                    else {
                        std::cout << "Failed to retrieve the frame!";
                    }
                }
            }
        }
        return FrameTextures;
    }
#pragma endregion 

#pragma region Connection
    void GetAvailableDevices() {
        //Wait for the PhoXiControl
        while (!Factory.isPhoXiControlRunning()) {
            LOCAL_CROSS_SLEEP(100);
        }
        std::cout << "PhoXi Control Version: " << Factory.GetPhoXiControlVersion() << std::endl;
        std::cout << "PhoXi API Version: " << Factory.GetAPIVersion() << std::endl;
        DeviceList = Factory.GetDeviceList();
        std::cout << "PhoXi Factory found " << DeviceList.size() << " devices by GetDeviceList call." << std::endl
            << std::endl;
        for (std::size_t i = 0; i < DeviceList.size(); i++) {
            std::cout << "Device: " << i << std::endl;
            std::cout << "  Name:                    " << DeviceList[i].Name << std::endl;
            std::cout << "  Hardware Identification: " << DeviceList[i].HWIdentification << std::endl;
            std::cout << "  Type:                    " << (std::string) DeviceList[i].Type << std::endl;
            std::cout << "  Firmware version:        " << DeviceList[i].FirmwareVersion << std::endl;
            std::cout << "  Variant:                 " << DeviceList[i].Variant << std::endl;
            std::cout << "  IsFileCamera:            " << (DeviceList[i].IsFileCamera ? "Yes" : "No") << std::endl;            
            std::cout << "  Status:                  " << (DeviceList[i].Status.Attached ? "Attached to PhoXi Control. "
                : "Not Attached to PhoXi Control. ")
                << (DeviceList[i].Status.Ready ? "Ready to connect" : "Occupied") << std::endl << std::endl;
        }
    }

    void ConnectPhoXiDeviceByPhoXiDeviceInformationEntry() {
        std::size_t Index;
        while (true) {
            std::cout << std::endl << "Please enter the Index listed from GetDeviceList call: ";
            if (!ReadLine(Index) || Index >= DeviceList.size()) {
                std::cout << "Bad Index, or not number!" << std::endl;
                continue;
            }
            break;
        }
        PhoXiDevice = Factory.Create(DeviceList[Index]);
        if (PhoXiDevice) {
            if (PhoXiDevice->Connect()) {
                std::cout << "Connection to the device " << DeviceList[Index].HWIdentification << " was Successful!"
                    << std::endl;
            }
            else {
                std::cout << "Connection to the device " << DeviceList[Index].HWIdentification << " was Unsuccessful!"
                    << std::endl;
            }
        }
        else {
            std::cout << "Unspecified error" << std::endl;
        }
    }

    void Disconnect() {
        std::cout << std::endl << std::endl;
        if (!AttachedFileCameraName.empty()) {
            Factory.DetachFileCamera(AttachedFileCameraName);
            AttachedFileCameraName = std::string();
        }
        if (PhoXiDevice && PhoXiDevice->isConnected()) {
            //The whole API is designed on C++ standards, using smart pointers and constructor/destructor logic
            //All resources will be closed automatically, but the device state will not be affected -> it will remain connected in PhoXi Control and if in freerun, it will remain Scanning
            //To Stop the device, just
            PhoXiDevice->StopAcquisition();
            //If you want to disconnect and logout the device from PhoXi Control, so it will then be available for other devices, call
            std::cout << "Do you want to logout the device? Enter 0 for no, enter 1 for yes: ";
            bool Entry;
            if (!ReadLine(Entry)) return;
            PhoXiDevice->Disconnect(Entry);
            //The call PhoXiDevice without Logout will be called automatically by destructor
        }
    }
#pragma endregion 

private:
    void PrintDepthMapInfo() const {
        std::cout << std::endl; std::cout << std::endl;
        std::cout << "You chose to test DepthMap." << std::endl;
    }
    DepthMapSettingsResult LoadDepthMapSettings() {
        // TODO: change the calibrationFilePath to point to the calibration.txt which was the result of the calibration process
        const auto calibrationFilePath = projectDirectory + delimiter + "calibration.txt";
        std::cout << calibrationFilePath << std::endl;

        std::ifstream stream(calibrationFilePath.c_str());
        if (!stream.good()) {
            return DepthMapSettingsResult::FileMissing;    
        }
        
        auto CorrectCalibration = true;
        DepthMapSetting.Calibration.LoadFromFile(calibrationFilePath);
        CorrectCalibration &= DepthMapSetting.Calibration.CalibrationSettings.DistortionCoefficients.size() > 4;
        CorrectCalibration &= DepthMapSetting.Calibration.CameraResolution.Width != 0 && DepthMapSetting.Calibration.CameraResolution.Height != 0;
        return CorrectCalibration
            ? DepthMapSettingsResult::Correct
            : DepthMapSettingsResult::Incorrect;
    }
    void PrintDepthMapSettings() {
        std::cout << "Loaded calibration:" << std::endl;
        std::cout << "Camera Matrix" << std::endl;
        for (size_t y = 0; y < DepthMapSetting.Calibration.CalibrationSettings.CameraMatrix.Size.Height; ++y) {
            for (size_t x = 0; x < DepthMapSetting.Calibration.CalibrationSettings.CameraMatrix.Size.Height; ++x) {
                std::cout << std::to_string(DepthMapSetting.Calibration.CalibrationSettings.CameraMatrix[y][x]) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "Distortion Coefficients" << std::endl;
        for (size_t i = 0; i < DepthMapSetting.Calibration.CalibrationSettings.DistortionCoefficients.size(); ++i) {
            std::cout << std::to_string(DepthMapSetting.Calibration.CalibrationSettings.DistortionCoefficients.at(i)) << " ";
        }
        std::cout << std::endl << std::endl;
        std::cout << "Rotation Matrix" << std::endl;
        for (size_t y = 0; y < DepthMapSetting.Calibration.CoordinateTransformation.Rotation.Size.Height; ++y) {
            for (size_t x = 0; x < DepthMapSetting.Calibration.CoordinateTransformation.Rotation.Size.Height; ++x) {
                std::cout << std::to_string(DepthMapSetting.Calibration.CoordinateTransformation.Rotation[y][x]) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "Translation vector" << std::endl;
        std::cout << std::to_string(DepthMapSetting.Calibration.CoordinateTransformation.Translation.x) << " ";
        std::cout << std::to_string(DepthMapSetting.Calibration.CoordinateTransformation.Translation.y) << " ";
        std::cout << std::to_string(DepthMapSetting.Calibration.CoordinateTransformation.Translation.z) << std::endl;
        std::cout << std::endl;
        std::cout << "Camera Resolution" << std::endl;
        std::cout << std::to_string(DepthMapSetting.Calibration.CameraResolution.Width) << "x" << std::to_string(DepthMapSetting.Calibration.CameraResolution.Height);
        std::cout << std::endl << std::endl;
    }
    void PrintDepthMapSettingsIncorrectError() const {
        std::cout << std::endl; std::cout << std::endl;
        std::cout << "Calibration saved in 'calibration.txt' is incorrect." << std::endl;
        std::cout << "Please make sure you have correct settings in 'Settings' folder and the 'MarkersPositions.txt' contains path to the file containing markers positions." << std::endl;
        std::cout << "After you have correct settings, start calibration again and upon success start the testing of depth map." << std::endl;
        std::cout << "Press any key to exit the application." << std::endl;
        std::string Output;
        ReadLine(Output);
    }
    void PrintDepthMapSettingsMissingError() const {
        std::cout << std::endl; std::cout << std::endl;
        const auto calibrationFilePath = 
            projectDirectory + delimiter + "calibration.txt";
        std::cout << "Calibration file 'calibration.txt' is missing." << std::endl;
        std::cout << "Application expects the 'calibration.txt' file to be on "
            "this path: " << calibrationFilePath << std::endl;
        std::cout << "CMake settings may change where the 'calibration.txt' "
            "file is created. Change CMake settings or copy 'calibration.txt' "
            "to the aformentioned path and restart the testing of depth map." << 
            std::endl;
        std::cout << "Press any key to exit the application." << std::endl;
        std::string Output;
        ReadLine(Output);
    }
    void PrintDepthMapFromFileOrConnectToScannerInfo() const {
        std::cout << "Please enter whether you want to load image from file or you want to connect to a scanner:" << std::endl;
        std::cout << "  1. Load from file" << std::endl;
        std::cout << "  2. Connect to a scanner" << std::endl;
    }
    void PrintDepthMapConnectToScannerInfo() const {
        std::cout << "You chose to connect to a scanner." << std::endl;
    }
    void PrintDepthMapShouldContinueInfo() const {
        std::cout << "Do you want to continue?" << std::endl;
        std::cout << "  1. Yes, trigger new scan and compute depth map" << std::endl;
        std::cout << "  2. No, disconnect the scanner" << std::endl;
    }

    void PrintCalibrationInfo() const {
        std::cout << std::endl; std::cout << std::endl;
        std::cout << "You chose to test the calibration process." << std::endl;
    }
    void PrintCalibrationConnectToScannerInfo() const {
        std::cout << "You chose to connect to a scanner." << std::endl;
    }
    void PrintCalibrationShouldContinueInfo() const {
        std::cout << std::endl; std::cout << std::endl;
        std::cout << "Do you want to continue?" << std::endl;
        std::cout << "  1. Yes, trigger a new scan a get image from external camera" << std::endl;
        std::cout << "  2. No, compute the calibration" << std::endl;
    }
    void PrintCalibrationFromFileInfo() const {
        std::cout << std::endl; std::cout << std::endl;
        std::cout << "You chose to load frames and images from files." << std::endl;
    }
    void PrintCalibrationFromFileOrConnectToScannerInfo() const {
        std::cout << "Please enter whether you want to load frames and images from files or you want to connect to a scanner:" << std::endl;
        std::cout << "  1. Load from files" << std::endl;
        std::cout << "  2. Connect to a scanner" << std::endl;
    }
    void PrintCalibrationSettings() const {
        std::cout << "Loaded following settings:" << std::endl;
        std::cout << "	Focal length: " << CalibrationSetting.FocalLength << " mm" << std::endl;
        std::cout << "	Pixel size: " << CalibrationSetting.PixelSize << " mm" << std::endl;
        std::cout << "	Markers positions: " << CalibrationSetting.MarkersPositions << std::endl << std::endl;
    }
    void PrintCalibrationOrDepthMapinfo() {
        std::cout << "PhoXi Control Version: " << Factory.GetPhoXiControlVersion() << std::endl;
        std::cout << "PhoXi API Version: " << Factory.GetAPIVersion() << std::endl << std::endl;
        std::cout << "Please enter the external camera test number which you want to preform:" << std::endl;
        std::cout << "  1. Test Calibration" << std::endl;
        std::cout << "  2. Test DepthMap" << std::endl;
        std::cout << "  3. Exit" << std::endl;
    }
    std::string GetProjectDirectory(const std::string& executablePath) {
        std::string projectDirectory;
#ifdef __linux__
        delimiter = "/";
#endif
#ifdef _WIN32
        delimiter = "\\";
#endif
#ifdef _WIN64
        delimiter = "\\";
#endif
        projectDirectory = executablePath.substr(0, executablePath.find_last_of(delimiter));
        projectDirectory = projectDirectory.substr(0, projectDirectory.find_last_of(delimiter));
#ifdef __linux__
        if (projectDirectory.size() <= 3) {
            char cwd[1024];
            if (getcwd(cwd, sizeof(cwd)) != NULL)
                projectDirectory = cwd;
        }
#endif
        std::cout << "Project directory identified as: " << projectDirectory << std::endl;
        return projectDirectory;
    }

    std::vector <PhoXiDeviceInformation> DeviceList;
    PPhoXi PhoXiDevice;
    PhoXiFactory Factory;
    CalibrationSettings CalibrationSetting;
    DepthMapSettings DepthMapSetting;
    std::string AttachedFileCameraName;
};

int main(int argc, char *argv[]) {
    if (argc > 1) {
        PhoXiExternalCameraExamples Example(argv[0], argv[1]);
    } else {
        PhoXiExternalCameraExamples Example(argv[0]);
    }
    return 0;
}

