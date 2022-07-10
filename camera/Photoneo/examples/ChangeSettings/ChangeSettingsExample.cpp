/*
* Photoneo's API Example - ChangeSettingsExample.cpp
* Defines the entry point for the console application.
* Demonstrates the extended functionality of PhoXi devices. This Example shows how to change the settings of the device.
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
class FullAPIExample
{
private:
	pho::api::PhoXiFactory Factory;
	pho::api::PPhoXi PhoXiDevice;
	pho::api::PFrame SampleFrame;
	std::vector<pho::api::PhoXiDeviceInformation> DeviceList;

	void CorrectDisconnectExample();
	void ConnectPhoXiDeviceBySerialExample();
	void PrintCapturingSettings(const pho::api::PhoXiCapturingSettings &CapturingSettings, const pho::api::PhoXiSize &Resolution);
	void ChangeCapturingSettingsExample();
	void ChangeProcessingSettingsExample();
	void ChangeCoordinatesSettingsExample();
	void CalibrationSettingsExample();
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

void FullAPIExample::ConnectPhoXiDeviceBySerialExample()
{
	std::cout << std::endl << "Please enter the Hardware Identification Number (for example 'YYYY-MM-###-LC#'): ";
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

void FullAPIExample::ChangeCapturingSettingsExample()
{
	std::cout << "Change Capturing Settings Example" << std::endl;
	std::cout << std::endl;
	//Retrieving the Current Capturing Settings
	pho::api::PhoXiCapturingSettings CurrentCapturingSettings;
	CurrentCapturingSettings = PhoXiDevice->CapturingSettings;
	//Check if the CurrentCapturingSettings have been retrieved succesfully
	if (!PhoXiDevice->CapturingSettings.isLastOperationSuccessful()) {
		throw std::runtime_error(PhoXiDevice->CapturingSettings.GetLastErrorMessage().c_str());
	}

	//Retrieving the Current Resolution
	pho::api::PhoXiSize CurrentResolution = PhoXiDevice->Resolution;
	//Check if the CurrentResolution has been retrieved succesfully
	if (!PhoXiDevice->Resolution.isLastOperationSuccessful())
	{
		throw std::runtime_error(PhoXiDevice->Resolution.GetLastErrorMessage().c_str());
	}

	//Printing the Current Capturing Settings
	std::cout << "Current Capturing Settings are the following:" << std::endl;
	PrintCapturingSettings(CurrentCapturingSettings, CurrentResolution);

	//ShutterMultiplier values: 1-20
	PhoXiDevice->CapturingSettings->ShutterMultiplier = 3;

	//ScanMultiplier values: 1-50
	PhoXiDevice->CapturingSettings->ScanMultiplier = 2;

	//Resolution values: 0 / 1 (0 is 2064x1544, 1 is 1032x772)
	//Get all supported modes
	std::vector<pho::api::PhoXiCapturingMode> SupportedCapturingModes = PhoXiDevice->SupportedCapturingModes;
	//Check if the SupportedCapturingModes have been retrieved succesfully
	if (!PhoXiDevice->SupportedCapturingModes.isLastOperationSuccessful())
	{
		throw std::runtime_error(PhoXiDevice->SupportedCapturingModes.GetLastErrorMessage().c_str());
	}
	PhoXiDevice->CapturingMode = SupportedCapturingModes[0];
	//Check if the Resolution has been changed succesfully
	if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
	{
		throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());
	}

	//CameraOnlyMode values: 0 / 1 or false / true (0 is OFF, 1 is ON)
	PhoXiDevice->CapturingSettings->CameraOnlyMode = false;

	//AmbientLightSuppression values: 0 / 1 or false / true (0 is OFF, 1 is ON)
	PhoXiDevice->CapturingSettings->AmbientLightSuppression = 1;

	//CodingStrategy values: Normal / Interreflections
	PhoXiDevice->CapturingSettings->CodingStrategy = "Normal";

	//CodingQuality values: Fast / High / Ultra
	PhoXiDevice->CapturingSettings->CodingQuality = "High";

	//TextureSource values: Computed / LED / Laser / Focus
	PhoXiDevice->CapturingSettings->TextureSource = "Laser";

	//SinglePatternExposure values: 10.24 / 14.336 / 20.48 / 24.576 / 30.72 / 34.816 / 40.96 / 49.152 / 75.776 / 79.872 / 90.112 / 100.352
	//Get all supported modes
	std::vector<double> SinglePatternExposures = PhoXiDevice->SupportedSinglePatternExposures;
	PhoXiDevice->CapturingSettings->SinglePatternExposure = SinglePatternExposures[4];

	//MaximumFPS values: 0-60
	PhoXiDevice->CapturingSettings->MaximumFPS = 1.12;

	//LaserPower values: possible from 0 - 4095, recommended 800 - 4095
	PhoXiDevice->CapturingSettings->LaserPower = 1650;

	//LEDPower values: possible from 0 - 4095, recommended 4095
	PhoXiDevice->CapturingSettings->LEDPower = 4095;
	//ProjectionOffsetLeft values: possible from 0 - 512
	PhoXiDevice->CapturingSettings->ProjectionOffsetLeft = 20;

	//ProjectionOffsetRight values: possible from 0 - 512,
	PhoXiDevice->CapturingSettings->ProjectionOffsetRight = 20;

	//Retrieving the Changed Capturing Settings
	pho::api::PhoXiCapturingSettings ChangedCapturingSettings;
	ChangedCapturingSettings = PhoXiDevice->CapturingSettings;
	pho::api::PhoXiSize ChangedResolution;
	//Getting Current Resolution
	if (PhoXiDevice->CapturingMode.isEnabled() && PhoXiDevice->CapturingMode.CanGet())
	{
		pho::api::PhoXiCapturingMode CapturingMode = PhoXiDevice->CapturingMode;
		//You can ask the feature, if the last performed operation was successful
		if (!PhoXiDevice->CapturingMode.isLastOperationSuccessful())
		{
			throw std::runtime_error(PhoXiDevice->CapturingMode.GetLastErrorMessage().c_str());
		}

		ChangedResolution = PhoXiDevice->Resolution;
	}
	std::cout << std::endl;
	std::cout << "Capturing Settings have been changed to the following:" << std::endl;
	PrintCapturingSettings(ChangedCapturingSettings, ChangedResolution);

	//Restore previous values
	PhoXiDevice->CapturingSettings = CurrentCapturingSettings;
}

void FullAPIExample::ChangeProcessingSettingsExample()
{
	std::cout << "Change Processing Settings Example" << std::endl;
	std::cout << std::endl;
	//Retrieving the Current Processing Settings
	pho::api::PhoXiProcessingSettings CurrentProcessingSettings;
	CurrentProcessingSettings = PhoXiDevice->ProcessingSettings;
	//Check if the CurrentProcessingSettings have been retrieved succesfully
	if (!PhoXiDevice->ProcessingSettings.isLastOperationSuccessful()) {
		throw std::runtime_error(PhoXiDevice->ProcessingSettings.GetLastErrorMessage().c_str());
	}
	std::cout << "Current Processing Settings are the following:" << std::endl;
	PrintProcessingSettings(CurrentProcessingSettings);

	//The example changes the settings to random values
	srand(time(NULL));

	//3D ROI Camera Space min/max
	//These variables are double so any value for a double are acceptable
	//values from -4000mm to 4000mm (maximum range of the PhoXi XL 3D scanner is 3780mm)
	PhoXiDevice->ProcessingSettings->ROI3D.CameraSpace.min.x = 0;
	PhoXiDevice->ProcessingSettings->ROI3D.CameraSpace.min.y = 40;
	PhoXiDevice->ProcessingSettings->ROI3D.CameraSpace.min.z = 60;
	PhoXiDevice->ProcessingSettings->ROI3D.CameraSpace.max.x = 2000;
	PhoXiDevice->ProcessingSettings->ROI3D.CameraSpace.max.y = 1000;
	PhoXiDevice->ProcessingSettings->ROI3D.CameraSpace.max.z = 3000;

	//3D ROI PointCloudSpace min/max
	//These variables are double so any value for a double are acceptable
	pho::api::Point3<double> ROI3DPointCloudSpaceMin, ROI3DPointCloudSpaceMax;
	ROI3DPointCloudSpaceMin.x = 8000 * ((double)rand() / RAND_MAX) - 4000; //from -4000mm to 4000mm (maximum range of the PhoXi XL 3D scanner is 3780mm)
	ROI3DPointCloudSpaceMin.y = 8000 * ((double)rand() / RAND_MAX) - 4000;
	ROI3DPointCloudSpaceMin.z = 8000 * ((double)rand() / RAND_MAX) - 4000;
	ROI3DPointCloudSpaceMax.x = 8000 * ((double)rand() / RAND_MAX) - 4000;
	ROI3DPointCloudSpaceMax.y = 8000 * ((double)rand() / RAND_MAX) - 4000;
	ROI3DPointCloudSpaceMax.z = 8000 * ((double)rand() / RAND_MAX) - 4000;
	PhoXiDevice->ProcessingSettings->ROI3D.PointCloudSpace.min = ROI3DPointCloudSpaceMin;
	PhoXiDevice->ProcessingSettings->ROI3D.PointCloudSpace.max = ROI3DPointCloudSpaceMax;

	//MaxCameraAngle values: 0-90
	PhoXiDevice->ProcessingSettings->NormalAngle.MaxCameraAngle = 90;

	//MaxProjectionAngle values: 0-90
	PhoXiDevice->ProcessingSettings->NormalAngle.MaxProjectorAngle = 80;

	//MinHalfwayAngle values: 0-90
	PhoXiDevice->ProcessingSettings->NormalAngle.MinHalfwayAngle = 10;

	//MaxHalfwayAngle values: 0-90
	PhoXiDevice->ProcessingSettings->NormalAngle.MaxHalfwayAngle = 0;

	//MaxInaccuracy(Confidence) values: 0-100
	PhoXiDevice->ProcessingSettings->Confidence = 2.0;

	//CalibrationVolumeCut values: 0 / 1 or false / true (0 is OFF, 1 is ON)
	PhoXiDevice->ProcessingSettings->CalibrationVolumeOnly = false;

	//SurfaceSmoothness values: Sharp / Normal / Smooth
	PhoXiDevice->ProcessingSettings->SurfaceSmoothness = "Normal";

	//NormalsEstimationRadius values: 1-4
	PhoXiDevice->ProcessingSettings->NormalsEstimationRadius = 2;

	//Retrieving the Changed Processing Settings
	pho::api::PhoXiProcessingSettings ChangedProcessingSettings;
	ChangedProcessingSettings = PhoXiDevice->ProcessingSettings;
	//Check if the CurrentProcessingSettings have been retrieved succesfully
	if (!PhoXiDevice->ProcessingSettings.isLastOperationSuccessful()) {
		throw std::runtime_error(PhoXiDevice->ProcessingSettings.GetLastErrorMessage().c_str());
	}
	std::cout << std::endl;
	std::cout << "Processing Settings  have been changed to the following:" << std::endl;
	PrintProcessingSettings(ChangedProcessingSettings);

	//Restore previous values
	PhoXiDevice->ProcessingSettings = CurrentProcessingSettings;
}

void FullAPIExample::ChangeCoordinatesSettingsExample() {
	std::cout << "Change Coordinates Settings Example" << std::endl;
	std::cout << std::endl;
	//Retrieving the Current Coordinates Settings
	pho::api::PhoXiCoordinatesSettings CurrentCoordinatesSettings;
	CurrentCoordinatesSettings = PhoXiDevice->CoordinatesSettings;
	//Check if the CurrentCoordinatesSettings have been retrieved succesfully
	if (!PhoXiDevice->CoordinatesSettings.isLastOperationSuccessful()) {
		throw std::runtime_error(PhoXiDevice->CoordinatesSettings.GetLastErrorMessage().c_str());
	}
	std::cout << "Current Coordinates Settings are the following:" << std::endl;
	PrintCoordinatesSettings(CurrentCoordinatesSettings);

	//Transformation from Camera Space to Mounting Space for PhoXi 3D scanner model L
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
	PhoXiDevice->CoordinatesSettings->CoordinateSpace = "CameraSpace";

	//CustomSpace Transformation
	PhoXiDevice->CoordinatesSettings->CustomTransformation = Transformation;

	//RobotSpace Transformation
	PhoXiDevice->CoordinatesSettings->RobotTransformation = Transformation;

	//Recognize Markers values: 0 / 1 or false / true (0 is OFF, 1 is ON)
	PhoXiDevice->CoordinatesSettings->RecognizeMarkers = 0;

	//Pattern Scale values: 0.0 - 1.0 (scale 1.0 x 1.0 is normal size)
	PhoXiDevice->CoordinatesSettings->MarkersSettings.MarkerScale = pho::api::PhoXiSize_64f(0.5, 0.5);

	//Retrieving the Changed Coordinates Settings
	pho::api::PhoXiCoordinatesSettings ChangedCoordinatesSettings;
	ChangedCoordinatesSettings = PhoXiDevice->CoordinatesSettings;
	//Check if the ChangedProcessingSettings have been retrieved succesfully
	if (!PhoXiDevice->CoordinatesSettings.isLastOperationSuccessful()) {
		throw std::runtime_error(PhoXiDevice->CoordinatesSettings.GetLastErrorMessage().c_str());
	}
	std::cout << std::endl;
	std::cout << "Changed Coordinates Settings are the following:" << std::endl;
	PrintCoordinatesSettings(ChangedCoordinatesSettings);

	//Restore previous values
	PhoXiDevice->CoordinatesSettings = CurrentCoordinatesSettings;
}

void FullAPIExample::CalibrationSettingsExample() {
	//Retrieving the CalibrationSettings 
	pho::api::PhoXiCalibrationSettings CalibrationSettings = PhoXiDevice->CalibrationSettings;
	//Check if the CalibrationSettings have been retrieved succesfully
	if (!PhoXiDevice->CalibrationSettings.isLastOperationSuccessful())
	{
		throw std::runtime_error(PhoXiDevice->CalibrationSettings.GetLastErrorMessage().c_str());
	}
	PrintCalibrationSettings(CalibrationSettings);
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

void FullAPIExample::PrintCapturingSettings(const pho::api::PhoXiCapturingSettings &CapturingSettings, const pho::api::PhoXiSize &Resolution)
{
	std::cout << "  CapturingSettings: " << std::endl;
	std::cout << "    ShutterMultiplier: " << CapturingSettings.ShutterMultiplier << std::endl;
	std::cout << "    ScanMultiplier: " << CapturingSettings.ScanMultiplier << std::endl;
	std::cout << "    Resolution: (" << Resolution.Width << "x" << Resolution.Height << ")" << std::endl;
	std::cout << "    CameraOnlyMode: " << CapturingSettings.CameraOnlyMode << std::endl;
	std::cout << "    AmbientLightSuppression: " << CapturingSettings.AmbientLightSuppression << std::endl;
	std::cout << "    CodingStrategy: " << std::string(CapturingSettings.CodingStrategy) << std::endl;
	std::cout << "    CodingQuality: " << std::string(CapturingSettings.CodingQuality) << std::endl;
	std::cout << "    TextureSource: " << std::string(CapturingSettings.TextureSource) << std::endl;
	std::cout << "    SinglePatternExposure: " << CapturingSettings.SinglePatternExposure << std::endl;
	std::cout << "    MaximumFPS: " << CapturingSettings.MaximumFPS << std::endl;
	std::cout << "    LaserPower: " << CapturingSettings.LaserPower << std::endl;
	std::cout << "    LEDPower: " << CapturingSettings.LEDPower << std::endl;
	std::cout << "    ProjectionOffsetLeft: " << CapturingSettings.ProjectionOffsetLeft << std::endl;
	std::cout << "    ProjectionOffsetRight: " << CapturingSettings.ProjectionOffsetRight << std::endl;
}

void FullAPIExample::PrintProcessingSettings(const pho::api::PhoXiProcessingSettings &ProcessingSettings)
{
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

void FullAPIExample::PrintCoordinatesSettings(const pho::api::PhoXiCoordinatesSettings &CoordinatesSettings)
{
	std::cout << "  CoordinatesSettings: " << std::endl;
	PrintMatrix("CustomRotationMatrix", CoordinatesSettings.CustomTransformation.Rotation);
	PrintVector("CustomTranslationVector", CoordinatesSettings.CustomTransformation.Translation);
	PrintMatrix("RobotRotationMatrix", CoordinatesSettings.RobotTransformation.Rotation);
	PrintVector("RobotTranslationVector", CoordinatesSettings.RobotTransformation.Translation);
	std::cout << "    CoordinateSpace: " << std::string(CoordinatesSettings.CoordinateSpace) << std::endl;
	std::cout << "    RecognizeMarkers: " << CoordinatesSettings.RecognizeMarkers << std::endl;
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
	{	//Connecting to scanner
		ConnectPhoXiDeviceBySerialExample();
		std::cout << std::endl;

		//Checks
		//Check if the device is connected
		if (!PhoXiDevice || !PhoXiDevice->isConnected())
		{
			std::cout << "Device is not created, or not connected!" << std::endl;
			return;
		}
		//Check if the CapturingSettings are Enabled and Can be Set
		if (!PhoXiDevice->CapturingSettings.isEnabled() || !PhoXiDevice->CapturingSettings.CanSet() || !PhoXiDevice->CapturingSettings.CanGet()) {
			std::cout << "Capturing Settings are not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
			return;
		}
		//Check if the CapturingModes are Enabled and Can be Set (needed to change the resolution)
		if (!PhoXiDevice->CapturingMode.isEnabled() || !PhoXiDevice->CapturingMode.CanGet() || !PhoXiDevice->CapturingMode.CanSet() || !PhoXiDevice->SupportedCapturingModes.isEnabled() || !PhoXiDevice->SupportedCapturingModes.CanGet())
		{
			std::cout << "Capturing Modes are not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
			return;
		}
		//End Checks
		
		ChangeCapturingSettingsExample();

		//Checks
		//Check if the ProcessingSettings are Enabled and Can be Set
		if (!PhoXiDevice->ProcessingSettings.isEnabled() || !PhoXiDevice->ProcessingSettings.CanSet() || !PhoXiDevice->ProcessingSettings.CanGet()) {
			std::cout << "ProcessingSettings not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
			return;
		}
		//End Checks

		std::cout << std::endl;
		ChangeProcessingSettingsExample();

		//Checks
		//Check if the CoordinatesSettings are Enabled and Can be Set
		if (!PhoXiDevice->CoordinatesSettings.isEnabled() || !PhoXiDevice->CoordinatesSettings.CanSet() || !PhoXiDevice->CoordinatesSettings.CanGet()) {
			std::cout << "CoordinatesSettings not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
			return;
		}
		//End Checks

		std::cout << std::endl;
		ChangeCoordinatesSettingsExample();
		
		//Checks
		//Check if the CoordinatesSettings are Enabled and Can be retrieved
		if (!PhoXiDevice->CalibrationSettings.isEnabled() || !PhoXiDevice->CalibrationSettings.CanGet())
		{
			std::cout << "CalibrationSettings not supported by the Device Hardware, or are Read only on the specific device" << std::endl;
			return;
		}
		//End Checks
		std::cout << std::endl;
		CalibrationSettingsExample();
		
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