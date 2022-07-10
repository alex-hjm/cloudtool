/*
* Photoneo's API Example - MotionCamChangeSettingsExample.cpp
* Defines the entry point for the console application.
* Demonstrates the extended functionality of MotionCam-3D devices.
* This Example shows how to change the settings of the device.
* Contains the usage of retrieving all parameters and how to change these parameters.
* Points out the correct way to disconnect the device from PhoXiControl.
*/

#include <ctime>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include "PhoXi.h"

//The whole api is in namespace pho (Photoneo) :: api
class MotionCamExample {
  private:
    pho::api::PhoXiFactory Factory;
    pho::api::PPhoXi Device;
    pho::api::PFrame SampleFrame;
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList;

    void CorrectDisconnectExample();
    void ConnectPhoXiDeviceBySerialExample();
    void ChangeMotionCamCameraModeExample();
    void ChangeMotionCamScannerModeExample();
    void ChangeMotionCam2DModeExample();
    void ChangeProcessingSettingsExample();
    void ChangeCoordinatesSettingsExample();
    void CalibrationSettingsExample();
    void PrintMotionCamGeneral(const pho::api::PhoXiMotionCam &MotionCam) const;
    void PrintMotionCamCameraMode(const pho::api::PhoXiMotionCamCameraMode &CameraMode) const;
    void PrintMotionCamScannerMode(const pho::api::PhoXiMotionCamScannerMode &ScannerMode) const;
    void PrintProcessingSettings(const pho::api::PhoXiProcessingSettings &ProcessingSettings) const;
    void PrintCoordinatesSettings(const pho::api::PhoXiCoordinatesSettings &CoordinatesSettings) const;
    void PrintCalibrationSettings(const pho::api::PhoXiCalibrationSettings &CalibrationSettings) const;
    void PrintResolution(const pho::api::PhoXiSize &Resolution) const;
    void PrintMatrix(const std::string &Name, const pho::api::CameraMatrix64f &Matrix) const;
    void PrintVector(const std::string &Name, const pho::api::Point3_64f &Vector) const;

    template<class T>
    bool ReadLine(T &Output) const {
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
    MotionCamExample() {};
    ~MotionCamExample() {};
    void Run();
};

void MotionCamExample::ConnectPhoXiDeviceBySerialExample() {
    std::cout << std::endl << "Please enter the Hardware Identification Number (for example 'YYYY-MM-###-LC#'): ";
    std::string HardwareIdentification;
    if (!ReadLine(HardwareIdentification)) {
        std::cout << "Incorrect input!" << std::endl;
        return;
    }

    pho::api::PhoXiTimeout Timeout = pho::api::PhoXiTimeout::ZeroTimeout;
    Device = Factory.CreateAndConnect(HardwareIdentification, Timeout);
    if (Device) {
        std::cout << "Connection to the device " << HardwareIdentification << " was Successful!" << std::endl;
    } else {
        std::cout << "Connection to the device " << HardwareIdentification << " was Unsuccessful!" << std::endl;
    }
    std::cout << std::endl;
}

void MotionCamExample::ChangeMotionCamCameraModeExample() {
    std::cout << std::endl;
    std::cout << "Change CameraMode Settings Example" << std::endl;

    //Make sure to have CameraMode selected
    Device->MotionCam->OperationMode = pho::api::PhoXiOperationMode::Camera;
    //Check if the CurrentMotionCamSettings have been set succesfully
    if (!Device->MotionCam.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->MotionCam.GetLastErrorMessage().c_str());
    }

    //Retrieving the Current Resolution from CapturingMode
    pho::api::PhoXiSize CurrentResolution = Device->CapturingMode->Resolution;
    //Check if the CurrentResolution has been retrieved succesfully
    if (!Device->CapturingMode.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->CapturingMode.GetLastErrorMessage().c_str());
    }

    //Printing the Current Resolution
    std::cout << "Current CameraMode Settings are the following:" << std::endl;
    PrintResolution(CurrentResolution);

    //Retrieving the CameraMode settings
    pho::api::PhoXiMotionCamCameraMode CurrentCameraMode = Device->MotionCamCameraMode.GetValue();
    //Check if the MotionCamCameraMode has been retrieved succesfully
    if (!Device->MotionCamCameraMode.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->MotionCamCameraMode.GetLastErrorMessage().c_str());
    }

    pho::api::PhoXiMotionCamCameraMode BackupCameraMode = CurrentCameraMode;

    PrintMotionCamCameraMode(CurrentCameraMode);

    //Exposure is ReadOnly
    //CurrentCameraMode.Exposure = 10.0;

    //SamplingTopology values: Standard
    CurrentCameraMode.SamplingTopology = pho::api::PhoXiSamplingTopology::Standard;

    //OutputTopology values: Raw / Irregular grid / Regular grid
    CurrentCameraMode.OutputTopology = pho::api::PhoXiOutputTopology::Raw;

    //CodingStrategy values: Normal / Interreflections
    CurrentCameraMode.CodingStrategy = pho::api::PhoXiCodingStrategy::Normal;

    //Set the CameraMode settings to the Device
    Device->MotionCamCameraMode.SetValue(CurrentCameraMode);
    //Check if the MotionCamCameraMode has been set succesfully
    if (!Device->MotionCamCameraMode.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->MotionCamCameraMode.GetLastErrorMessage().c_str());
    }

    std::cout << "CameraMode Settings have been changed to the following:" << std::endl;
    PrintMotionCamCameraMode(CurrentCameraMode);

    //Restore previous values
    Device->MotionCamCameraMode.SetValue(BackupCameraMode);
}

void MotionCamExample::ChangeMotionCamScannerModeExample() {
    std::cout << std::endl;
    std::cout << "Change ScannerMode Settings Example" << std::endl;

    //Make sure to have ScannerMode selected
    Device->MotionCam->OperationMode = pho::api::PhoXiOperationMode::Scanner;
    //Check if the CurrentMotionCamSettings have been set succesfully
    if (!Device->MotionCam.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->MotionCam.GetLastErrorMessage().c_str());
    }

    //Retrieving the Current Resolution from CapturingMode
    pho::api::PhoXiSize CurrentResolution = Device->CapturingMode->Resolution;
    //Check if the CurrentResolution has been retrieved succesfully
    if (!Device->CapturingMode.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->CapturingMode.GetLastErrorMessage().c_str());
    }

    //Printing the Current Resolution
    std::cout << "Current ScannerMode Settings are the following:" << std::endl;
    PrintResolution(CurrentResolution);

    //Retrieving the CameraMode settings
    pho::api::PhoXiMotionCamScannerMode CurrentScannerMode = Device->MotionCamScannerMode.GetValue();
    //Check if the MotionCamScannerMode has been retrieved succesfully
    if (!Device->MotionCamScannerMode.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->MotionCamScannerMode.GetLastErrorMessage().c_str());
    }

    pho::api::PhoXiMotionCamScannerMode BackupScannerMode = CurrentScannerMode;

    PrintMotionCamScannerMode(CurrentScannerMode);

    //ShutterMultiplier values: 0-10
    CurrentScannerMode.ShutterMultiplier = 2;

    //ScanMultiplier values: 0-10
    CurrentScannerMode.ScanMultiplier = 2;

    //CodingStrategy values: Normal / Interreflections
    CurrentScannerMode.CodingStrategy = pho::api::PhoXiCodingStrategy::Normal;

    //CodingQuality values: Fast / High / Ultra
    CurrentScannerMode.CodingQuality = pho::api::PhoXiCodingQuality::Fast;

    //TextureSource values: LED / Computed / Laser / Focus
    CurrentScannerMode.TextureSource = pho::api::PhoXiTextureSource::LED;

    //Set the ScannerMode settings to the Device
    Device->MotionCamScannerMode.SetValue(CurrentScannerMode);
    //Check if the MotionCamScannerMode has been set succesfully
    if (!Device->MotionCamScannerMode.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->MotionCamScannerMode.GetLastErrorMessage().c_str());
    }

    std::cout << "ScannerMode Settings have been changed to the following:" << std::endl;
    PrintMotionCamScannerMode(CurrentScannerMode);

    //Restore previous values
    Device->MotionCamScannerMode.SetValue(BackupScannerMode);
}

void MotionCamExample::ChangeMotionCam2DModeExample() {
    std::cout << std::endl;
    std::cout << "Change 2D Mode Settings Example" << std::endl;

    //Make sure to have Mode2D selected
    Device->MotionCam->OperationMode = pho::api::PhoXiOperationMode::Mode2D;
    //Check if the CurrentMotionCamSettings have been set succesfully
    if (!Device->MotionCam.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->MotionCam.GetLastErrorMessage().c_str());
    }

    //Retrieving the Current Resolution from CapturingMode
    pho::api::PhoXiSize CurrentResolution = Device->CapturingMode->Resolution;
    //Check if the CurrentResolution has been retrieved succesfully
    if (!Device->CapturingMode.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->CapturingMode.GetLastErrorMessage().c_str());
    }

    //Printing the Current Resolution
    std::cout << "Current 2D Mode Settings are the following:" << std::endl;
    PrintResolution(CurrentResolution);
}

void MotionCamExample::ChangeProcessingSettingsExample() {
    std::cout << std::endl;
    std::cout << "Change Processing Settings Example" << std::endl;

    //Retrieving the Current Processing Settings
    pho::api::PhoXiProcessingSettings CurrentProcessingSettings = Device->ProcessingSettings.GetValue();
    //Check if the CurrentProcessingSettings have been retrieved succesfully
    if (!Device->ProcessingSettings.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->ProcessingSettings.GetLastErrorMessage().c_str());
    }
    std::cout << "Current Processing Settings are the following:" << std::endl;
    PrintProcessingSettings(CurrentProcessingSettings);

    pho::api::PhoXiProcessingSettings BackupProcessingSettings = CurrentProcessingSettings;

    //The example changes the settings to random values
    srand(time(NULL));

    //3D ROI Camera Space min/max
    //These variables are double so any value for a double are acceptable
    CurrentProcessingSettings.ROI3D.CameraSpace.min.x = 0;
    CurrentProcessingSettings.ROI3D.CameraSpace.min.y = 40;
    CurrentProcessingSettings.ROI3D.CameraSpace.min.z = 60;
    CurrentProcessingSettings.ROI3D.CameraSpace.max.x = 2000;
    CurrentProcessingSettings.ROI3D.CameraSpace.max.y = 1000;
    CurrentProcessingSettings.ROI3D.CameraSpace.max.z = 3000;

    //3D ROI PointCloudSpace min/max
    //These variables are double so any value for a double are acceptable
    pho::api::Point3<double> ROI3DPointCloudSpaceMin, ROI3DPointCloudSpaceMax;
    ROI3DPointCloudSpaceMin.x = 8000 * ((double)rand() / RAND_MAX) - 4000;
    ROI3DPointCloudSpaceMin.y = 8000 * ((double)rand() / RAND_MAX) - 4000;
    ROI3DPointCloudSpaceMin.z = 8000 * ((double)rand() / RAND_MAX) - 4000;
    CurrentProcessingSettings.ROI3D.PointCloudSpace.min = ROI3DPointCloudSpaceMin;
    ROI3DPointCloudSpaceMax.x = 8000 * ((double)rand() / RAND_MAX) - 4000;
    ROI3DPointCloudSpaceMax.y = 8000 * ((double)rand() / RAND_MAX) - 4000;
    ROI3DPointCloudSpaceMax.z = 8000 * ((double)rand() / RAND_MAX) - 4000;
    CurrentProcessingSettings.ROI3D.PointCloudSpace.max = ROI3DPointCloudSpaceMax;

    //MaxCameraAngle values: 0-90
    CurrentProcessingSettings.NormalAngle.MaxCameraAngle = 90;

    //MaxProjectionAngle values: 0-90
    CurrentProcessingSettings.NormalAngle.MaxProjectorAngle = 80;

    //MinHalfwayAngle values: 0-90
    CurrentProcessingSettings.NormalAngle.MinHalfwayAngle = 10;

    //MaxHalfwayAngle values: 0-90
    CurrentProcessingSettings.NormalAngle.MaxHalfwayAngle = 0;

    //MaxInaccuracy(Confidence) values: 0-100
    CurrentProcessingSettings.Confidence = 2.0;

    //CalibrationVolumeCut values: 0 / 1 or false / true (0 is OFF, 1 is ON)
    CurrentProcessingSettings.CalibrationVolumeOnly = false;

    //SurfaceSmoothness values: Sharp / Normal / Smooth
    CurrentProcessingSettings.SurfaceSmoothness = "Normal";

    //NormalsEstimationRadius values: 1-4
    CurrentProcessingSettings.NormalsEstimationRadius = 2;

    //Set the ProcessingSettings settings to the Device
    Device->ProcessingSettings.SetValue(CurrentProcessingSettings);
    //Check if the ProcessingSettings has been set succesfully
    if (!Device->ProcessingSettings.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->ProcessingSettings.GetLastErrorMessage().c_str());
    }

    std::cout << "Processing Settings have been changed to the following:" << std::endl;
    PrintProcessingSettings(CurrentProcessingSettings);

    //Restore previous values
    Device->ProcessingSettings.SetValue(BackupProcessingSettings);
}

void MotionCamExample::ChangeCoordinatesSettingsExample() {
    std::cout << std::endl;
    std::cout << "Change Coordinates Settings Example" << std::endl;

    //Retrieving the Current Coordinates Settings
    pho::api::PhoXiCoordinatesSettings CurrentCoordinatesSettings = Device->CoordinatesSettings.GetValue();

    //Check if the CurrentCoordinatesSettings have been retrieved succesfully
    if (!Device->CoordinatesSettings.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->CoordinatesSettings.GetLastErrorMessage().c_str());
    }
    std::cout << "Current Coordinates Settings are the following:" << std::endl;
    PrintCoordinatesSettings(CurrentCoordinatesSettings);

    pho::api::PhoXiCoordinatesSettings BackupCoordinatesSettings = CurrentCoordinatesSettings;

    pho::api::PhoXiCoordinateTransformation Transformation;
    Transformation.Rotation.At(0, 0) = -0.986429;
    Transformation.Rotation.At(0, 1) = 0;
    Transformation.Rotation.At(0, 2) = 0.164187;
    Transformation.Rotation.At(1, 0) = 0;
    Transformation.Rotation.At(1, 1) = -1;
    Transformation.Rotation.At(1, 2) = 0;
    Transformation.Rotation.At(2, 0) = 0.164187;
    Transformation.Rotation.At(2, 1) = 0;
    Transformation.Rotation.At(2, 2) = 0.986429;
    Transformation.Translation.x = -271.97;
    Transformation.Translation.y = 29.9;
    Transformation.Translation.z = 36.14;

    //CoordinateSpace values: CameraSpace / MarkerSpace / RobotSpace / CustomSpace
    CurrentCoordinatesSettings.CoordinateSpace = "CameraSpace";

    //CustomSpace Transformation
    CurrentCoordinatesSettings.CustomTransformation = Transformation;

    //RobotSpace Transformation
    CurrentCoordinatesSettings.RobotTransformation = Transformation;

    //Recognize Markers values: 0 / 1 or false / true (0 is OFF, 1 is ON)
    CurrentCoordinatesSettings.RecognizeMarkers = 0;

    //Pattern Scale values: 0.0 - 1.0 (scale 1.0 x 1.0 is normal size)
    CurrentCoordinatesSettings.MarkersSettings.MarkerScale = pho::api::PhoXiSize_64f(0.5, 0.5);

    //Set the CoordinatesSettings settings to the Device
    Device->CoordinatesSettings.SetValue(CurrentCoordinatesSettings);
    //Check if the CoordinatesSettings has been set succesfully
    if (!Device->CoordinatesSettings.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->CoordinatesSettings.GetLastErrorMessage().c_str());
    }

    std::cout << "Coordinates Settings have been changed to the following:" << std::endl;
    PrintCoordinatesSettings(CurrentCoordinatesSettings);

    //Restore previous values
    Device->CoordinatesSettings.SetValue(BackupCoordinatesSettings);
}

void MotionCamExample::CalibrationSettingsExample() {
    std::cout << std::endl;
    std::cout << "Get Calibration Settings Example" << std::endl;

    //Retrieving the CalibrationSettings
    pho::api::PhoXiCalibrationSettings CalibrationSettings = Device->CalibrationSettings.GetValue();
    //Check if the CalibrationSettings have been retrieved succesfully
    if (!Device->CalibrationSettings.isLastOperationSuccessful()) {
        throw std::runtime_error(Device->CalibrationSettings.GetLastErrorMessage().c_str());
    }
    PrintCalibrationSettings(CalibrationSettings);
}

void MotionCamExample::CorrectDisconnectExample() {
    //The whole API is designed on C++ standards, using smart pointers and constructor/destructor logic
    //All resources will be closed automatically, but the device state will not be affected -> it will remain connected in PhoXi Control and if in freerun, it will remain Scanning
    //To Stop the device, just
    Device->StopAcquisition();
    //If you want to disconnect and logout the device from PhoXi Control, so it will then be available for other devices, call
    std::cout << "Do you want to logout the device? (0 if NO / 1 if YES) ";
    bool Entry;
    if (!ReadLine(Entry)) {
        return;
    }
    Device->Disconnect(Entry);
    //The call PhoXiDevice without Logout will be called automatically by destructor
}

void MotionCamExample::PrintMotionCamGeneral(const pho::api::PhoXiMotionCam &MotionCam) const {
    std::cout << "  MotionCam: " << std::endl;
    std::cout << "    OperationMode: " << std::string(MotionCam.OperationMode) << std::endl;
    std::cout << "    LaserPower: " << MotionCam.LaserPower << std::endl;
    std::cout << "    MaximumFPS: " << MotionCam.MaximumFPS << std::endl;
}

void MotionCamExample::PrintMotionCamCameraMode(const pho::api::PhoXiMotionCamCameraMode &CameraMode) const {
    std::cout << "  CameraMode: " << std::endl;
    std::cout << "    Exposure: " << CameraMode.Exposure << std::endl;
    std::cout << "    SamplingTopology: " << std::string(CameraMode.SamplingTopology) << std::endl;
    std::cout << "    OutputTopology: " << std::string(CameraMode.OutputTopology) << std::endl;
    std::cout << "    CodingStrategy: " << std::string(CameraMode.CodingStrategy) << std::endl;
}

void MotionCamExample::PrintMotionCamScannerMode(const pho::api::PhoXiMotionCamScannerMode &ScannerMode) const {
    std::cout << "  ScannerMode: " << std::endl;
    std::cout << "    ShutterMultiplier: " << ScannerMode.ShutterMultiplier << std::endl;
    std::cout << "    ScanMultiplier: " << ScannerMode.ScanMultiplier << std::endl;
    std::cout << "    CodingStrategy: " << std::string(ScannerMode.CodingStrategy) << std::endl;
    std::cout << "    CodingQuality: " << std::string(ScannerMode.CodingQuality) << std::endl;
    std::cout << "    TextureSource: " << std::string(ScannerMode.TextureSource) << std::endl;
}

void MotionCamExample::PrintProcessingSettings(const pho::api::PhoXiProcessingSettings &ProcessingSettings) const {
    std::cout << "  ProcessingSettings: " << std::endl;
    PrintVector("MinCameraSpace(in DataCutting)", ProcessingSettings.ROI3D.CameraSpace.min);
    PrintVector("MaxCameraSpace(in DataCutting)", ProcessingSettings.ROI3D.CameraSpace.max);
    PrintVector("MinPointCloudSpace (in DataCutting)", ProcessingSettings.ROI3D.PointCloudSpace.min);
    PrintVector("MaxPointCloudSpace (in DataCutting)", ProcessingSettings.ROI3D.PointCloudSpace.max);
    std::cout << "    MaxCameraAngle: " << ProcessingSettings.NormalAngle.MaxCameraAngle << std::endl;
    std::cout << "    MaxProjectionAngle: " << ProcessingSettings.NormalAngle.MaxProjectorAngle << std::endl;
    std::cout << "    MinHalfwayAngle: " << ProcessingSettings.NormalAngle.MinHalfwayAngle << std::endl;
    std::cout << "    MaxHalfwayAngle: " << ProcessingSettings.NormalAngle.MaxHalfwayAngle << std::endl;
    std::cout << "    Confidence (MaxInaccuracy): " << ProcessingSettings.Confidence << std::endl;
    std::cout << "    SurfaceSmoothness: " << std::string(ProcessingSettings.SurfaceSmoothness) << std::endl;
    std::cout << "    NormalsEstimationRadius: " << ProcessingSettings.NormalsEstimationRadius << std::endl;
}

void MotionCamExample::PrintCoordinatesSettings(const pho::api::PhoXiCoordinatesSettings &CoordinatesSettings) const {
    std::cout << "  CoordinatesSettings: " << std::endl;
    PrintMatrix("CustomRotationMatrix", CoordinatesSettings.CustomTransformation.Rotation);
    PrintVector("CustomTranslationVector", CoordinatesSettings.CustomTransformation.Translation);
    PrintMatrix("RobotRotationMatrix", CoordinatesSettings.RobotTransformation.Rotation);
    PrintVector("RobotTranslationVector", CoordinatesSettings.RobotTransformation.Translation);
    std::cout << "    CoordinateSpace: " << std::string(CoordinatesSettings.CoordinateSpace) << std::endl;
    std::cout << "    RecognizeMarkers: " << CoordinatesSettings.RecognizeMarkers << std::endl;
}

void MotionCamExample::PrintCalibrationSettings(const pho::api::PhoXiCalibrationSettings &CalibrationSettings) const {
    std::cout << "  CalibrationSettings: " << std::endl;
    std::cout << "    FocusLength: " << CalibrationSettings.FocusLength << std::endl;
    std::cout << "    PixelSize: "
        << CalibrationSettings.PixelSize.Width
        << " x "
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
    for (int i = 1; i < distCoeffs.size(); ++i) {
        if (i == 4 || i == 5 || i == 8 || i == 12 || i == 14) {
            currentDistCoeffsSS << "[";
            ++brackets;
        }
        currentDistCoeffsSS << ", " << distCoeffs[i];
    }
    for (int j = 0; j < brackets; ++j) {
        currentDistCoeffsSS << "]";
    }
    currentDistCoeffsSS << ")";
    std::cout << "      " << currentDistCoeffsSS.str() << std::endl;
}

void MotionCamExample::PrintResolution(const pho::api::PhoXiSize &Resolution) const {
    std::cout << "Resolution: ("
        << Resolution.Width
        << "x"
        << Resolution.Height
        << ")" << std::endl;
}

void MotionCamExample::PrintVector(const std::string &name, const pho::api::Point3_64f &vector) const {
    std::cout << "    " << name << ": ["
        << vector.x << "; "
        << vector.y << "; "
        << vector.z << "]"
        << std::endl;
}

void MotionCamExample::PrintMatrix(const std::string &name, const pho::api::CameraMatrix64f &matrix) const {
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

void MotionCamExample::Run() {
    try {
        //Connecting to device
        ConnectPhoXiDeviceBySerialExample();
        std::cout << std::endl;

        //Check if the device is connected
        if (!Device || !Device->isConnected()) {
            std::cout << "Device is not created, or not connected!" << std::endl;
            return;
        }

        //Check if the MotionCam are Enabled and Can be Set
        if (!Device->MotionCam.isEnabled() || !Device->MotionCam.CanSet() || !Device->MotionCam.CanGet()) {
            std::cout << "MotionCam not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
            return;
        }
        ChangeMotionCam2DModeExample();

        //Check if the MotionCamCameraMode are Enabled and Can be Set
        if (!Device->MotionCamCameraMode.isEnabled() || !Device->MotionCamCameraMode.CanSet() || !Device->MotionCamCameraMode.CanGet()) {
            std::cout << "MotionCamCameraMode not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
            return;
        }
        ChangeMotionCamCameraModeExample();

        //Check if the MotionCamScannerMode are Enabled and Can be Set
        if (!Device->MotionCamScannerMode.isEnabled() || !Device->MotionCamScannerMode.CanSet() || !Device->MotionCamScannerMode.CanGet()) {
            std::cout << "MotionCamScannerMode not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
            return;
        }
        ChangeMotionCamScannerModeExample();

        //Check if the ProcessingSettings are Enabled and Can be Set
        if (!Device->ProcessingSettings.isEnabled() || !Device->ProcessingSettings.CanSet() || !Device->ProcessingSettings.CanGet()) {
            std::cout << "ProcessingSettings not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
            return;
        }
        ChangeProcessingSettingsExample();

        //Check if the CoordinatesSettings are Enabled and Can be Set
        if (!Device->CoordinatesSettings.isEnabled() || !Device->CoordinatesSettings.CanSet() || !Device->CoordinatesSettings.CanGet()) {
            std::cout << "CoordinatesSettings not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
            return;
        }
        ChangeCoordinatesSettingsExample();

        //Check if the CoordinatesSettings are Enabled and Can be retrieved
        if (!Device->CalibrationSettings.isEnabled() || !Device->CalibrationSettings.CanGet()) {
            std::cout << "CalibrationSettings not supported by the Device Hardware" << std::endl;
            return;
        }
        CalibrationSettingsExample();

        CorrectDisconnectExample();
    }
    catch (std::runtime_error &InternalException) {
        std::cout << std::endl << "Exception was thrown: " << InternalException.what() << std::endl;
        if (Device->isConnected()) {
            Device->Disconnect(true);
        }
    }
}

int main(int argc, char *argv[]) {
    MotionCamExample Example;
    Example.Run();
    return 0;
}
