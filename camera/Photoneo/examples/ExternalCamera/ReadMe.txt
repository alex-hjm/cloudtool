========================================================================
    CONSOLE APPLICATION : ExternalCameraExample Project Overview
========================================================================

ExternalCameraExample is a console application which is used to compute 
calibration for external camera and to get the aligned depth map from the 
point of view of external camera.

You will learn how to:

* calibrate the Scanner to work with an external 2D camera,
* obtain depth map aligned from the point of view of the external camera,
* align color texture from external camera with point cloud.

Prerequisites
- Download external_camera_example_1.2.zip from http://photoneo.com/files/installer/PhoXi/api/external_camera_example_1.2.zip
and extract it to ExternalCameraExample_CPP folder. This folder 
contains all needed data for testing of calibration using file camera.
You need to have the Data folder present in ExternalCameraExample_CPP folder
before running CMake
- Make sure you have opencv 3.1.0 installed on correct path:
  - Windows: C:/opencv/
  - Linux: home/OpenCV/
  - Or modify CMake if you have it installed on different path
  - You can also change opencv version on your own risk
- CMake ExternalCameraExample

Running the example application
- You can add optional command line parameter which represents the absolute path 
to 'Settings' folder.
- This folder needs to contain PixelSize.txt, FocalLength.txt, 
MarkersPositions.txt
- PixelSize.txt  = needs to contain one double value of pixel size of external 
camera in mm
- FocalLength.txt = needs to contain one double value of focal length of external
camera in mm
- MarkersPositions.txt = needs to contain markers positions
- If no command line parameter is supplied, default 'Settings' folder will be set
to '{PROJECT_DIRECTORY}\Settings\'.

Workflow
Application asks whether you want to test the calibration process or compute the 
aligned depth map. Press 1 for calibration, press 2 for depth map, press 3 to 
exit the application.


Calibration
If you chose calibration: Focal length, Pixel size and Markers positions will 
be loaded into Calibration Settings. If you started ExternalCameraExample with
command line parameter with the absolute path to 'Settings' folder, the 
application will load the values from that folder.
By default the 'Settings' folder is set to {PROJECT_DIRECTORY}.
If you want to change the values, you can do it in: 
{Settings}/FocalLength.txt
{Settings}/PixelSize.txt
{Settings}/MarkersPositions.txt
Important! Write Focal length and pixel size in millimeters. 
Into MarkersPositions.txt write markers positions.

After loading the calibration settings, the application asks whether you want 
to load frames (scans from device camera unit) and images (images from external
camera) from files or you want to connect to a scanner. 
Press 1 for load from files, press 2 to connect to a scanner.

Load from files
If you chose to load frames and images from files. The application will load 10
test frames and 10 test images from files which can be found in:
{PROJECT_FOLDER}/Data/
Frames have a prefix of ‘frame’. Images have a prefix of ‘image’.
After successfully loading frames and images the application will connect to a 
file camera included in the project.
When the connection to the file camera is successful, the application will 
start the calibration process.
The result of calibration can be found in:
{PROJECT_FOLDER}/calibration.txt
The structure of calibration.txt is as follows:
  Camera Matrix – 9 double values separated with whitespace
  Distortion Coefficients – 5 to 14 double values separated with whitespace
  Rotation Matrix – 9 double values separated with whitespace
  Translation Vector – 3 double values separated with whitespace
  Camera Resolution – 2 values separated with whitespace (width, height)

Connect to a scanner
Before using this option you need to implement the following method in code:
cv::Mat GetImageFromAdditionalCamera()
This method must be implemented by you to get the image from external camera. 
After you implemented it, you can restart the process and select this option.
The application will show a list of available scanners and prompt the user to 
connect to one. You can connect to a scanner by pressing the corresponding key.

After successful connection the application will ask you whether you want to 
trigger a scan. Press 1 to trigger a scan or 2 to stop the process.
When the scan arrives, it will be saved to calibration settings and application
will make a call to GetImageFromAdditionalCamera which will get an image from 
the external camera (remember that you need to implement this method yourself).
You can repeat the process by pressing 1 and additional scan will be triggered 
and image from external camera will be retrieved. The process needs at least 5 
scans to compute the calibration.
When you press 2 the calibration will automatically start and process is the 
same as in the previous step (Load from files).


Depth Map
If you chose to test depth map, Calibration settings will be loaded from 
calibration.txt
Loaded settings will be printed out before computing the depth map.
After the settings are printed out the application asks whether you want to 
compute the aligned depth map from file or you want to connect to a scanner.

From file
If you chose to compute aligned depth map from file, the application will 
connect to a test file camera included in the project
After successful connection the application will trigger a scan.
At last the application will start to compute the aligned depth map.
The result depth map can be found in:
{PROJECT_FOLDER}/fileCamera_1.jpg

Connect to a scanner
If you choose to connect to a scanner the application will show a list of 
available scanners and prompt the user to connect to one. You can connect 
to a scanner by pressing the corresponding key.
After successful connection to scanner the application will ask you whether 
you want to trigger a scan. Press 1 to trigger a scan or 2 to stop the process.
When the scan arrives aligned depth map will be calculated and saved in the 
project folder as device_1.jpg.
The application will ask whether you want to continue the process. You can 
continue the process by pressing 1. 
The next aligned depth maps will be saved as:
{PROJECT_FOLDER}/device_2.jpg, {PROJECT_FOLDER}/device_3.jpg and so on.

/////////////////////////////////////////////////////////////////////////////
