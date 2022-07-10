#include "PhoXi.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <chrono>
#include <thread>

#if defined(_WIN32)
#include <windows.h>
#elif defined (__linux__)
#include <unistd.h>
#endif

using namespace std;

const string FILENAME_POSTFIX = "_coordinates.txt";

// list all physical scanners on the network
vector<pho::api::PhoXiDeviceInformation> listDevices(pho::api::PhoXiFactory factory);
// uses vector from listDevices
int promtPickScanner(vector<pho::api::PhoXiDeviceInformation> deviceList);

pho::api::PPhoXi connectToScanner(int scannerIndex, vector <pho::api::PhoXiDeviceInformation> deviceList,
    pho::api::PhoXiFactory factory);
bool areCoordinateSettingsSupported(pho::api::PPhoXi device);
// cli for saving and loading transformation files
bool promtUser(pho::api::PPhoXi device);
// triggers the scan and saves custom transformation
bool writeConfig(pho::api::PPhoXi device, string configFile);
// triggers the scan and applies custom transformation
bool readConfig(pho::api::PPhoXi device, string configFile);
// extracts homogeneous transformation matrix from FrameInfo into simple 1D array
void getSimpleMatrix(const pho::api::FrameInfo info, double result[16]);
// transforms matrix from our simple 1D array representation into PhoXiCoordinateTransformation
pho::api::PhoXiCoordinateTransformation getAPIMatrix(const double simpleMatrix[16]);
// saves 1D array matrix
bool saveSimpleMatrix(double simpleMatrix[16], string file);
// reads 1D array matrix
bool loadSimpleMatrix(string file, double mat[16]);
// returns true for file cameras
bool isFileCamera(pho::api::PhoXiDeviceInformation device);
// true if simpleMatrix could be transformation matrix, false for identity matrix indicating error
bool isValidMatrix(double simpleMatrix[16]);


int main(int argc, char *argv[]) {
    pho::api::PhoXiFactory  phoxiFactory;
    auto deviceList = listDevices(phoxiFactory);
    char scannerIndex = promtPickScanner(deviceList);
    auto device = connectToScanner(scannerIndex, deviceList, phoxiFactory);

    if (device && device->isConnected() && areCoordinateSettingsSupported(device))
    {
        device->TriggerMode = pho::api::PhoXiTriggerMode::Software;

        while (promtUser(device))
        {
            cout << endl;
        }
    }
    else
    {
        cout << "Did not connect, closing" << endl;
        cout << "Press q and enter to quit" << endl;
        char a;
        std::cin >> a;
        return -1;
    }

    return 0;
}

bool promtUser(pho::api::PPhoXi device)
{
    cout << "\nUsage:\n\
    'w filename': write current configuration to filename_coordinates.txt\n\
    'r filename': read configuration from filename_coordinates.txt\n\
    'q': quit application\n";

    char command;
    string filename;
    cin >> command;

    switch (command) {
    case 'w':
    case 'W':
        cin >> filename;
        cout << "Processing..." << endl;
        writeConfig(device, filename + FILENAME_POSTFIX);
        cout << "Done" << endl;

        break;
    case 'r':
    case 'R':
        cin >> filename;
        cout << "Processing..." << endl;
        readConfig(device, filename + FILENAME_POSTFIX);
        cout << "Done" << endl;
        break;
    case 'q':
    case 'Q':
        return false;
    default:
        cout << "Command not recognized" << endl;
        cin.ignore();
        break;
    }

    return true;
}

bool areCoordinateSettingsSupported(pho::api::PPhoXi device)
{
    if (device->Features.isFeatureSupported("CoordinatesSettings"))
    {
        return true;
    }

    cout << "Feature CoordinatesSettings is not supported. Make sure you have " <<
        "PhoXiControl version 1.2.1 or higher" << endl;
    return false;
}

bool writeConfig(pho::api::PPhoXi device, string configFile)
{
    if (!device->isConnected())
        return false;
    if (!device->isAcquiring())
        device->StartAcquisition();

    auto MarkerSpaceSetting = pho::api::PhoXiCoordinatesSettings();
    MarkerSpaceSetting.CoordinateSpace = pho::api::PhoXiCoordinateSpace::MarkerSpace;
    MarkerSpaceSetting.MarkersSettings.InvertedMarkers = true;
    MarkerSpaceSetting.RecognizeMarkers = true;
    bool result = device->CoordinatesSettings.SetValue(MarkerSpaceSetting);
    device->TriggerFrame();
    auto frame = device->GetFrame();
    double mat[16];
    getSimpleMatrix(frame->Info, mat);
    if (!isValidMatrix(mat))
    {
        cout << "Did not locate markers, didn't save configuration" << endl;
        return false;
    }
    result &= saveSimpleMatrix(mat, configFile);
    return result;
}

bool readConfig(pho::api::PPhoXi device, string configFile)
{
    if (!device->isConnected())
    {
        cout << "Scnner not connected, please reconnect via PhoXi Contoll app or restart this project" << endl;
        return false;
    }
    if (!device->isAcquiring())
        device->StartAcquisition();

    auto CustomSpaceSetting = pho::api::PhoXiCoordinatesSettings();
    auto TransformMatrix = pho::api::PhoXiCoordinateTransformation();
    TransformMatrix.Rotation;
    TransformMatrix.Translation;
    CustomSpaceSetting.CoordinateSpace = pho::api::PhoXiCoordinateSpace::CustomSpace;
    double simpleMatrix[16];
    bool valid = loadSimpleMatrix(configFile, simpleMatrix);
    if (!valid)
    {
        return false;
    }

    CustomSpaceSetting.CustomTransformation = getAPIMatrix(simpleMatrix);
    bool result = device->CoordinatesSettings.SetValue(CustomSpaceSetting);
    device->TriggerFrame();
    device->GetFrame(); // wait until the frame arrives


    return result;
}

vector<pho::api::PhoXiDeviceInformation> listDevices(pho::api::PhoXiFactory factory)
{
    bool firstMessage = true;
    factory.isPhoXiControlRunning();
    while (!factory.isPhoXiControlRunning())
    {
        if (firstMessage)
        {
            firstMessage = false;
            cout << "PhoXi Control App not running.\n";
        }
        chrono::duration<long, milli> sleepPeriod(800);
        this_thread::sleep_for(sleepPeriod);
    }
    uint16_t index = 0;
    auto deviceList = factory.GetDeviceList();
    cout << "Available devices:" << endl;
    for (auto deviceInfo : deviceList)
    {
        if (!isFileCamera(deviceInfo)) // disregard file cameras
        {
            cout << "\t" << index << " [" << deviceInfo.HWIdentification
                << "] :(" << deviceInfo.Name << ") - "
                << ((deviceInfo.Status.Ready) ? "Ready" : "Occupied") << endl;
        }
        index++;
    }
    return deviceList;
}

void getSimpleMatrix(const pho::api::FrameInfo info, double result[16])
{
    result[0] = info.SensorXAxis.x;
    result[1] = info.SensorYAxis.x;
    result[2] = info.SensorZAxis.x;
    result[3] = info.SensorPosition.x;

    result[4] = info.SensorXAxis.y;
    result[5] = info.SensorYAxis.y;
    result[6] = info.SensorZAxis.y;
    result[7] = info.SensorPosition.y;

    result[8] = info.SensorXAxis.z;
    result[9] = info.SensorYAxis.z;
    result[10] = info.SensorZAxis.z;
    result[11] = info.SensorPosition.z;

    result[12] = 0;
    result[13] = 0;
    result[14] = 0;
    result[15] = 1;
}

pho::api::PhoXiCoordinateTransformation getAPIMatrix(const double simpleMatrix[16])
{
    pho::api::PhoXiCoordinateTransformation result;
    result.Rotation.At(0, 0) = simpleMatrix[0];
    result.Rotation.At(0, 1) = simpleMatrix[1];
    result.Rotation.At(0, 2) = simpleMatrix[2];
    result.Translation.x = simpleMatrix[3];

    result.Rotation.At(1, 0) = simpleMatrix[4];
    result.Rotation.At(1, 1) = simpleMatrix[5];
    result.Rotation.At(1, 2) = simpleMatrix[6];
    result.Translation.y = simpleMatrix[7];

    result.Rotation.At(2, 0) = simpleMatrix[8];
    result.Rotation.At(2, 1) = simpleMatrix[9];
    result.Rotation.At(2, 2) = simpleMatrix[10];
    result.Translation.z = simpleMatrix[11];

    return result;
}

bool saveSimpleMatrix(double simpleMatrix[16], string file)
{
    ofstream f(file, ofstream::out);

    for (int i = 0; i < 16; i++)
    {
        f << simpleMatrix[i];
        f << ((i % 4 == 3) ? '\n' : ' ');
    }
    f.close();
    return true;
}

bool loadSimpleMatrix(string file, double mat[16])
{
    ifstream f(file, ifstream::in);
    if (!(f && f.good()))
    {
        cout << file << " doesn't exist" << endl;
        return false;
    }
    for (int i = 0; i < 16; i++)
    {
        f >> mat[i];
        if (!f.good())
        {
            cout << "config file incomplete" << endl;
            return false;
        }
    }
    f.close();
    if (!isValidMatrix(mat))
    {
        cout << "Config load error: " << file << "contains invalid transformation, probably due to error in configuration saving" << endl;
        return false;
    }
    return true;
}

bool isFileCamera(pho::api::PhoXiDeviceInformation device)
{
    return device.FirmwareVersion == "";
}

bool isValidMatrix(double simpleMatrix[16])
{
    return simpleMatrix[0] != 1.0 && simpleMatrix[11] != 0.0;
}

int promtPickScanner(vector<pho::api::PhoXiDeviceInformation> deviceList)
{
    int index = -1;
    cout << "Insert device id or [HWIdentification],\n for example '1' or '[1611005]':" << endl;
    string line;
    getline(cin, line);
    pho::api::PPhoXi device;
    if (line[0] == '[')
    {
        string id = line.substr(1, line.size() - 2);
        for (auto i = 0; i < deviceList.size(); i++)
        {
            if (deviceList[i].Type == pho::api::PhoXiDeviceType::PhoXiScanner
                && deviceList[i].HWIdentification == id)
            {
                index = i;
                break;
            }
        }
    }
    else {
        try {
            index = stoi(line);

        }
        catch (invalid_argument& e)
        {
            cout << "Not valid: " << line << endl;
            return -1;
        }

    }

    return index;
}

pho::api::PPhoXi connectToScanner(int scannerIndex, vector <pho::api::PhoXiDeviceInformation> deviceList,
    pho::api::PhoXiFactory factory) {
    if (scannerIndex < 0)
    {
        cout << "Index out of range :" << scannerIndex << endl;
        return false;
    }
    if (scannerIndex > deviceList.size())
    {
        cout << "Camera with index " << scannerIndex << " is undefined" << endl;
        return false;
    }

    if (isFileCamera(deviceList[scannerIndex]))
    {
        cout << deviceList[scannerIndex].HWIdentification << " is file camera, pick physical camera" << endl;
    }
    auto device = factory.Create(deviceList[scannerIndex]);

    if (device->CanConnect())
        device->Connect();

    return device;
}
