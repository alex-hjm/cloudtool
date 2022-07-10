/*
* Photoneo's API Example - ApplyCustomProjectionExample.cpp
* Defines the entry point for the console application.
* Demonstrates the extended functionality of PhoXi devices. This Example shows how to change camera space to custom space
* and reproject depth map to this custom space and save it.
*/

#include "PhoXi.h"
#include <opencv2/opencv.hpp>
#include "PhoXiOpenCVSupport.h"

//The whole api is in namespace pho (Photoneo) :: api
class ApplyCustomProjectionExample {
private:
    pho::api::PhoXiFactory Factory;
    pho::api::PPhoXi PhoXiDevice;

    void CorrectDisconnect();
    void ConnectPhoXiDeviceBySerial();
    void ChangeAxisRotation(double alpha, double beta, double gamma);
    void ChangeCameraAngleExample();

    template<class T>
    bool ReadLine(T& Output) const {
        std::string Input;
        std::getline(std::cin, Input);
        std::stringstream InputSteam(Input);
        return (InputSteam >> Output) ? true : false;
    }
    bool ReadLine(std::string& Output) const {
        std::getline(std::cin, Output);
        return true;
    }

    double Angle2Radian(double angle) const {
        return angle * 3.14159265 / 180.;
    }

public:
    ApplyCustomProjectionExample() {};
    ~ApplyCustomProjectionExample() {};
    void Run();
};

void ApplyCustomProjectionExample::ConnectPhoXiDeviceBySerial() {
    std::cout << std::endl << "Please enter the Hardware Identification Number (for example 'YYYY-MM-###-LC#'): ";
    std::string HardwareIdentification;
    if (!ReadLine(HardwareIdentification)) {
        std::cout << "Incorrect input!" << std::endl;
        return;
    }

    pho::api::PhoXiTimeout Timeout = pho::api::PhoXiTimeout::ZeroTimeout;
    PhoXiDevice = Factory.CreateAndConnect(HardwareIdentification, Timeout);
    if (PhoXiDevice) {
        std::cout << "Connection to the device " << HardwareIdentification << " was Successful!" << std::endl;
    } else {
        std::cout << "Connection to the device " << HardwareIdentification << " was Unsuccessful!" << std::endl;
    }
}

void ApplyCustomProjectionExample::ChangeAxisRotation(double alpha, double beta, double gamma) {
    //Transformation around T axis matrix
    pho::api::PhoXiCoordinateTransformation Transformation;
    //1st row
    Transformation.Rotation.At(0, 0) = cos(gamma) * cos(beta);
    Transformation.Rotation.At(0, 1) = -sin(gamma) * cos(beta);
    Transformation.Rotation.At(0, 2) = sin(beta);
    //2nd row
    Transformation.Rotation.At(1, 0) = cos(gamma) * sin(beta) * sin(alpha) + sin(gamma) * cos(alpha);
    Transformation.Rotation.At(1, 1) = cos(gamma) * cos(alpha) - sin(gamma) * sin(beta) * sin(alpha);
    Transformation.Rotation.At(1, 2) = -cos(beta) * sin(alpha);
    //3rd row
    Transformation.Rotation.At(2, 0) = sin(gamma) * sin(alpha) - cos(gamma) * sin(beta) * cos(alpha);
    Transformation.Rotation.At(2, 1) = sin(gamma) * sin(beta) * cos(alpha) + sin(alpha) * cos(gamma);
    Transformation.Rotation.At(2, 2) = cos(beta) * cos(alpha);
    //4th row, translation vector
    Transformation.Translation.x = 0;
    Transformation.Translation.y = 0;
    Transformation.Translation.z = 0;

    //CoordinateSpace values: We must select CustomSpace to take into account transformation matrix
    PhoXiDevice->CoordinatesSettings->CoordinateSpace = "CustomSpace";

    //CustomSpace Transformation
    PhoXiDevice->CoordinatesSettings->CustomTransformation = Transformation;
}

void ApplyCustomProjectionExample::CorrectDisconnect() {
    //The whole API is designed on C++ standards, using smart pointers and constructor/destructor logic
    //All resources will be closed automatically, but the device state will not be affected -> it will remain connected in PhoXi Control and if in freerun, it will remain Scanning
    //To Stop the device, just
    PhoXiDevice->StopAcquisition();
    //If you want to disconnect and logout the device from PhoXi Control, so it will then be available for other devices, call
    std::cout << "Do you want to logout the device? (0 if NO / 1 if YES) ";
    bool Entry;
    if (!ReadLine(Entry)) {
        return;
    }
    PhoXiDevice->Disconnect(Entry);
    //The call PhoXiDevice without Logout will be called automatically by destructor
}

void ApplyCustomProjectionExample::Run() {
    try {
        //Connecting to scanner
        ConnectPhoXiDeviceBySerial();
        std::cout << std::endl;

        //Checks
        //Check if the device is connected
        if (!PhoXiDevice || !PhoXiDevice->isConnected()) {
            std::cout << "Device is not created, or not connected!" << std::endl;
            return;
        }

        //Checks
        //Check if the CoordinatesSettings are Enabled and Can be Set
        if (!PhoXiDevice->CoordinatesSettings.isEnabled() || !PhoXiDevice->CoordinatesSettings.CanSet() || !PhoXiDevice->CoordinatesSettings.CanGet()) {
            std::cout << "CoordinatesSettings not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
            return;
        }
        //End Checks

        ChangeCameraAngleExample();

        CorrectDisconnect();
    } catch (std::runtime_error& InternalException) {
        std::cout << std::endl << "Exception was thrown: " << InternalException.what() << std::endl;
        if (PhoXiDevice->isConnected()) {
            PhoXiDevice->Disconnect(true);
        }
    }
}

void ApplyCustomProjectionExample::ChangeCameraAngleExample() {
    //trigger first, camera space frame
    auto cameraSpaceFrameId = PhoXiDevice->TriggerFrame();
    if (cameraSpaceFrameId < 0) {
        std::cout << "Trigger camera space frame was not successful" << std::endl;
        return;
    }
    else {
        std::cout << "Camera space frame ID: " << cameraSpaceFrameId << std::endl;
    }
    pho::api::PFrame frame = PhoXiDevice->GetFrame();

    //Uncomment your scanner model
    //S model
    //ChangeAxisRotation(Angle2Radian(0.), Angle2Radian(-15.45)/*sensor angle for model S*/, Angle2Radian(0.));
    //M model
    ChangeAxisRotation(Angle2Radian(0.), Angle2Radian(-11.75)/*sensor angle for model M*/, Angle2Radian(0.));
    //L model
    //ChangeAxisRotation(Angle2Radian(0.), Angle2Radian(-9.45)/*sensor angle for model L*/, Angle2Radian(0.));
    //XL model
    //ChangeAxisRotation(Angle2Radian(0.), Angle2Radian(-7.5)/*sensor angle for model XL*/, Angle2Radian(0.));

    //trigger second, custom rotated frame
    auto customSpaceFrameId = PhoXiDevice->TriggerFrame();
    if (customSpaceFrameId < 0) {
        std::cout << "Trigger custom rotated frame was not successful" << std::endl;
        return;
    }
    else {
        std::cout << "Custom space frame ID: " << customSpaceFrameId << std::endl;
    }
    frame = PhoXiDevice->GetFrame();

    //load camera calibration parameters
    const pho::api::PhoXiCalibrationSettings& calibrationSettings = PhoXiDevice->CalibrationSettings;
    pho::api::float32_t k1, k2, p1, p2, k3, k4, k5, k6;
    k1 = calibrationSettings.DistortionCoefficients[0];
    k2 = calibrationSettings.DistortionCoefficients[1];
    p1 = calibrationSettings.DistortionCoefficients[2];
    p2 = calibrationSettings.DistortionCoefficients[3];
    k3 = calibrationSettings.DistortionCoefficients[4];
    k4 = calibrationSettings.DistortionCoefficients[5];
    k5 = calibrationSettings.DistortionCoefficients[6];
    k6 = calibrationSettings.DistortionCoefficients[7];

    pho::api::float32_t fx, fy, cx, cy;
    fx = calibrationSettings.CameraMatrix[0][0];
    fy = calibrationSettings.CameraMatrix[1][1];
    cx = calibrationSettings.CameraMatrix[0][2];
    cy = calibrationSettings.CameraMatrix[1][2];

    pho::api::float32_t zMin = std::numeric_limits<pho::api::float32_t>::max(), zMax = 0;

    //Zero-point
    pho::api::Point3_32f ZeroPoint(0.0f, 0.0f, 0.0f);

    //create depth map of required size
    pho::api::DepthMap32f depthMap(frame->PointCloud.Size);

    std::cout << "Creating depth map..." << std::endl;

    //Loop through the PointCloud
    for (int y = 0; y < frame->PointCloud.Size.Height; ++y) {
        for (int x = 0; x < frame->PointCloud.Size.Width; ++x) {
            //current point
            auto currentPt = frame->PointCloud[y][x];

            //Do the computation for a valid point only
            if (currentPt == ZeroPoint)
                continue;

            pho::api::float32_t z = currentPt.z;
            pho::api::float32_t xt = currentPt.x / z;
            pho::api::float32_t yt = currentPt.y / z;

            //find z min max
            zMin = MIN(zMin, z);
            zMax = MAX(zMax, z);

            pho::api::float32_t r2 = xt * xt + yt * yt;
            pho::api::float32_t r4 = r2 * r2;
            pho::api::float32_t r6 = r4 * r2;

            //these constants takes care of the radial distortion
            pho::api::float32_t c1 = (1 + k1 * r2 + k2 * r4 + k3 * r6);
            pho::api::float32_t c2 = (1 + k4 * r2 + k5 * r4 + k6 * r6);

            pho::api::float32_t xtt = xt * (c1 / c2) + 2 * p1 * xt * yt + p2 * (r2 + 2 * xt * xt);
            pho::api::float32_t ytt = yt * (c1 / c2) + p1 * (r2 + 2 * yt * yt) + 2 * p2 * xt * yt;

            pho::api::float32_t u = xtt * fx + cx;
            pho::api::float32_t v = ytt * fy + cy;

            int ui = (int)(u + 0.5);
            int vi = (int)(v + 0.5);

            if (ui < 0 || ui >= depthMap.Size.Width)
                continue;

            if (vi < 0 || vi >= depthMap.Size.Height)
                continue;

            //select closer value
            if (fabs(depthMap.At(vi, ui)) <= z) {
                depthMap.At(vi, ui) = z;
            }
        }
    }

    double range = zMax - zMin;
    double rangeConstant = range / 255.;
    pho::api::DepthMap32f depthMapAutoMinMax(depthMap);

    std::cout << "Creating auto minmax depth map..." << std::endl;

    //apply auto min max -> rerange values to <0;255> interval
    for (int y = 0; y < depthMapAutoMinMax.Size.Height; ++y) {
        for (int x = 0; x < depthMapAutoMinMax.Size.Width; ++x) {
            depthMapAutoMinMax.At(y, x) -= zMin;
            depthMapAutoMinMax.At(y, x) /= rangeConstant;
        }
    }

    cv::Mat ResultDepthMap;

    //converting and save depth maps
    if (!ConvertMat2DToOpenCVMat(depthMap, ResultDepthMap)) {
        std::cout << "Converting depth map was not successful" << std::endl;
        return;
    }
    auto err = cv::imwrite("depthMap.png", ResultDepthMap);
    if (cv::imwrite("depthMap.png", ResultDepthMap)) {
        std::cout << "depthMap.png saved" << std::endl;
    }
    else {
        std::cout << "saving depthMap.png was not successful" << std::endl;
        return;
    }

    if (!ConvertMat2DToOpenCVMat(depthMapAutoMinMax, ResultDepthMap)) {
        std::cout << "Converting depth map was not successful" << std::endl;
        return;
    }
    if (cv::imwrite("depthMapAutoMinMax.png", ResultDepthMap)) {
        std::cout << "depthMapAutoMinMax.png saved" << std::endl;
    }
    else {
        std::cout << "saving depthMapAutoMinMax.png was not successful" << std::endl;
        return;
    }
}

int main(int argc, char* argv[]) {
    ApplyCustomProjectionExample Example;
    Example.Run();
    return 0;
}