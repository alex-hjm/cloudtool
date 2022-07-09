#pragma once
#ifndef _PHOXI_FEATURE_TYPES_H
#define _PHOXI_FEATURE_TYPES_H
#include "PhoXiCompilerDefines.h"


#include <memory>
#include <string>
#ifdef PHOXI_PCL_SUPPORT
#  include "PhoXiPCLSupport.h"
#endif
#ifdef PHOXI_OPENCV_SUPPORT
#  include "PhoXiOpenCVSupport.h"
#endif
#include "PhoXiDataTypes.h"

namespace pho {
namespace api {

#pragma pack(push, 1)

#pragma pack(1)
///Object defining the capturing mode of the device.
/**
Settings inside the structure affect the data size of the output and will
rebuild the Cyclic buffer. To get supported capturing modes, call
SupportedCapturingModes. No other settings are supported.
*/
class PHOXI_DLL_API PhoXiCapturingMode {
  public:
    /**
    The Resolution will affect the Hardware of the sensor, with lower
    resolution, you can increase the S/N ratio and improve the per point
    scanning quality and speed if high resolution is not a necessity.
    Lower resolution will increase speed and decrease capturing noise.
    */
    PhoXiSize Resolution;
    PhoXiCapturingMode();
    PhoXiCapturingMode(const PhoXiCapturingMode &Other);
    PhoXiCapturingMode &operator=(const PhoXiCapturingMode &Other);
    bool operator==(const PhoXiCapturingMode &Other) const;
};

#pragma pack(1)
///Stores the status of the device.
class PHOXI_DLL_API PhoXiStatus {
  public:
    ///Tells if the device is Attached to PhoXi Control.
    bool Attached;
    ///Tells if the device is connected to the API.
    bool Connected;
    ///Tells if the device is in Acquisition mode.
    bool Acquiring;
    PhoXiStatus();
    bool operator=(const PhoXiStatus &Other);
};

//Forward Declaration for internal purposes
class PhoXiTriggerModeInternal;
#pragma pack(1)
///Defines the trigger mode of the device.
/**
Currently, only Freerun and Software Trigger are supported. The type is also
used as a smart enumerator. PhoXiDevice->TriggerMode = PhoXiTriggerMode::Software.
*/
class PHOXI_DLL_API PhoXiTriggerMode {
  public:
    enum Value {
        Freerun = 0,
        Software = 1,
        Hardware = 2,
        NoValue = 3
    };
  private:
    friend class PhoXiTriggerModeInternal;
    Value TriggerMode;
  public:
    PhoXiTriggerMode();
    PhoXiTriggerMode(const PhoXiTriggerMode &Other);
    PhoXiTriggerMode(const Value &TriggerMode);
    PhoXiTriggerMode(const int &Other);
    bool operator=(const PhoXiTriggerMode &Other);
    bool operator=(const Value &TriggerMode);
    bool operator=(const int &TriggerMode);
    bool operator=(const std::string &TriggerMode);
    operator Value() const;
    operator std::string() const;
};

//Forward Declaration for internal purposes
class PhoXiTriggerSignalModeInternal;
#pragma pack(1)
///Not supported.
class PHOXI_DLL_API PhoXiTriggerSignalMode {
  public:
    enum Value {
        Rising = 0,
        Falling = 1,
        NoValue = 2
    };
  private:
    friend class PhoXiTriggerSignalModeInternal;
    Value TriggerSignalMode;
  public:
    PhoXiTriggerSignalMode();
    PhoXiTriggerSignalMode(const Value &TriggerSignalMode);
    bool operator=(const Value &TriggerSignalMode);
    bool operator=(const std::string &TriggerSignalMode);
    operator Value() const;
    operator std::string() const;
};

class PhoXiTimeoutInternal;
#pragma pack(1)
///Timeout format in ms.
/**
PhoXiTimeout is used in all calls when Timeout have sense calls Like GetFrame
takes Timeout as optional parameters to wait for the frame. If default param is
supplied, timeout supplied as PhoXiDevice->Timeout will be used.
*/
class PHOXI_DLL_API PhoXiTimeout {
  public:
    enum Value {
        ZeroTimeout = 0,
        Infinity = -1,
        LastStored = -2,
        Default = -3
    };
  private:
    friend class PhoXiTimeoutInternal;
    int Timeout;
  public:
    PhoXiTimeout();
    PhoXiTimeout(const Value &Timeout);
    PhoXiTimeout(const int &Timeout);
    PhoXiTimeout(const std::string &Timeout);
    bool operator=(const Value &Timeout);
    bool operator=(const int &Timeout);
    bool operator=(const std::string &Timeout);
    bool operator==(const PhoXiTimeout &Other) const;
    bool operator==(const Value &Other) const;
    bool operator==(const int &Other) const;
    operator int() const;
    operator std::string() const;
    bool isNum() const;
};

//Forward Declaration for internal purposes
class PhoXiCoordinateSpaceInternal;
#pragma pack(1)
///Defines the used coordinate space.
/**
Currently, only Freerun and Software Trigger are supported. The type is also
used as a smart enumerator. PhoXiDevice->CoordinateSettings->CoordinateSpace
= PhoXiCoordinateSpace::CameraSpace.
*/
class PHOXI_DLL_API PhoXiCoordinateSpace {
  public:
    enum Value {
        NoValue = 0,
        CameraSpace = 1,
        //MountingSpace = 2,
        MarkerSpace = 3,
        RobotSpace = 4,
        CustomSpace = 5
    };
  private:
    friend class PhoXiCoordinateSpaceInternal;
    Value CoordinateSpace;
  public:
    PhoXiCoordinateSpace();
    PhoXiCoordinateSpace(const PhoXiCoordinateSpace &Other);
    PhoXiCoordinateSpace(const Value &CoordinateSpace);
    PhoXiCoordinateSpace(const int &Other);
    bool operator=(const PhoXiCoordinateSpace &Other);
    bool operator=(const Value &CoordinateSpace);
    bool operator=(const int &CoordinateSpace);
    bool operator=(const std::string &CoordinateSpace);
    bool operator==(const PhoXiCoordinateSpace &Other) const;
    bool operator==(const Value &Other) const;
    operator Value() const;
    operator std::string() const;
};

#pragma pack(1)
///Internal settings for the Photoneo's Markers recognition algorithm.
class PHOXI_DLL_API PhoXiCoordinatesMarkersSettings {
  public:
    ///If true, the algorithm will search for white circles on black background.
    bool InvertedMarkers;
    ///Marker pattern scale (scale 1.0 is the original size)
    PhoXiSize_64f MarkerScale;
    PhoXiCoordinatesMarkersSettings();
    bool operator=(const PhoXiCoordinatesMarkersSettings &Other);
    bool operator==(const PhoXiCoordinatesMarkersSettings &Other) const;
};

#pragma pack(1)
///Settings affecting coordinate space, supported by PhoXi Control 1.2 and up.
class PHOXI_DLL_API PhoXiCoordinatesSettings {
  public:
    ///Internal settings for the Photoneo's Markers recognition algorithm.
    PhoXiCoordinatesMarkersSettings MarkersSettings;
    ///Defines the used Coordinate Space of the device.
    PhoXiCoordinateSpace CoordinateSpace;
    ///Custom Coordinate transformation.
    PhoXiCoordinateTransformation CustomTransformation;
    ///Robot Coordinate transformation.
    PhoXiCoordinateTransformation RobotTransformation;
    /**
    Defines if the Recognize Markers feature is activated. If this feature
    is activated - It will try to recognize Photoneo's Marker field.
    */
    bool RecognizeMarkers;
    PhoXiCoordinatesSettings();
    bool operator==(const PhoXiCoordinatesSettings &Other) const;
};

#pragma pack(1)
///Device output setup.
/**
Select the desired output. All chosen outputs needs to be transported from the
Device and affect latency. PointCloud and DepthMap use shared structure, Both
selected had the same effect (speaking of data package) as one or the other.
*/
struct PHOXI_DLL_API FrameOutputSettings {
    bool SendPointCloud;
    bool SendNormalMap;
    bool SendDepthMap;
    bool SendConfidenceMap;
    bool SendTexture;
    FrameOutputSettings();
    FrameOutputSettings(const FrameOutputSettings &Other);
    FrameOutputSettings &operator=(const FrameOutputSettings &Other);
    bool operator==(const FrameOutputSettings &Other) const;
};

//Forward Declaration for internal purposes
class PhoXiCodingStrategyInternal;
#pragma pack(1)
///Defines the used coding strategy in projected patterns.
/**
Select a desired Coding Strategy
Coding strategy optimized for Interreflections use advance digital coding to
make diffuse inter-reflection possible to suppress. For most scenes, strategy
optimized for Interreflections provides better output, but in some edge cases
of glossy inter-reflections, Normal could provide more robust results. Default
is Interreflections.
*/
class PHOXI_DLL_API PhoXiCodingStrategy {
  public:
    enum Value {
        NoValue = 0,
        Normal = 1,
        Interreflections = 2,
    };
  private:
    friend class PhoXiCodingStrategyInternal;
    Value CodingStrategy;
  public:
    PhoXiCodingStrategy();
    PhoXiCodingStrategy(const PhoXiCodingStrategy &Other);
    PhoXiCodingStrategy(const Value &CodingStrategy);
    PhoXiCodingStrategy(const int &Other);
    bool operator==(const PhoXiCodingStrategy &Other) const;
    bool operator==(const Value &Other) const;
    bool operator=(const PhoXiCodingStrategy &Other);
    bool operator=(const Value &CodingStrategy);
    bool operator=(const int &CodingStrategy);
    bool operator=(const std::string &CodingStrategy);
    operator Value() const;
    operator std::string() const;
};

//Forward Declaration for internal purposes
class PhoXiCodingQualityInternal;
#pragma pack(1)
///Defines the used coding quality.
/**
Select a desired Coding Quality. This parameter influences the scanning time.
Default is High.
*/
class PHOXI_DLL_API PhoXiCodingQuality {
  public:
    enum Value {
        NoValue = 0,
        Fast = 1,
        High = 2,
        Ultra = 3,
    };
  private:
    friend class PhoXiCodingQualityInternal;
    Value CodingQuality;
  public:
    PhoXiCodingQuality();
    PhoXiCodingQuality(const PhoXiCodingQuality &Other);
    PhoXiCodingQuality(const Value &CodingQuality);
    PhoXiCodingQuality(const int &Other);
    bool operator==(const PhoXiCodingQuality &Other) const;
    bool operator==(const Value &Other) const;
    bool operator=(const PhoXiCodingQuality &Other);
    bool operator=(const Value &CodingQuality);
    bool operator=(const int &CodingQuality);
    bool operator=(const std::string &CodingQuality);
    operator Value() const;
    operator std::string() const;
};

//Forward Declaration for internal purposes
class PhoXiTextureSourceInternal;
#pragma pack(1)
///Defines the used texture source.
/**
Select a source of the texture. Computed is a texture computed from the
structured patterns - no additional image is necessary -> Lower Quality. LED
will trigger an additional image with LED flash that offers an speckle less
2D image. Laser will trigger an additional image with Laser flash. Focus will
set a structured pattern as a texture. Useful for analyzing problems with
signal contrast and the setup of the shutter parameters. Color option is only
available for Scanners with an additional color camera. Default is LED.
*/
class PHOXI_DLL_API PhoXiTextureSource {
  public:
    enum Value {
        NoValue = 0,
        Computed = 1,
        LED = 2,
        Laser = 3,
        Focus = 4,
        Color = 5
    };
  private:
    friend class PhoXiTextureSourceInternal;
    Value TextureSource;
  public:
    PhoXiTextureSource();
    PhoXiTextureSource(const PhoXiTextureSource &Other);
    PhoXiTextureSource(const Value &TextureSource);
    PhoXiTextureSource(const int &Other);
    bool operator==(const PhoXiTextureSource &Other) const;
    bool operator==(const Value &Other) const;
    bool operator=(const PhoXiTextureSource &Other);
    bool operator=(const Value &TextureSource);
    bool operator=(const int &TextureSource);
    bool operator=(const std::string &TextureSource);
    operator Value() const;
    operator std::string() const;
};

#pragma pack(1)
///Defines data cutting ROIs.
/**
Defines data cutting volumes in either Camera, or PointCloud space. Cutting the
data based on 3D component in CameraSpace helps cut depending on camera
coordinate space, while ROI defined in PointCloud Space can be used for cut only
data inside a specific box, or on top of Marker Space... The filtration will
only be applied if max > min and is applied individually for any dimension
e.g. you can specify CameraSpace.min.z to 500 and CameraSpace.max.z to 1000 to
filter all data that are outside of this z region.
*/
class PHOXI_DLL_API PhoXi3DROI {
  public:
    AxisVolume_64f CameraSpace;
    AxisVolume_64f PointCloudSpace;
    PhoXi3DROI();
    PhoXi3DROI(const PhoXi3DROI &Other);
    PhoXi3DROI &operator=(const PhoXi3DROI &Other);
    bool operator==(const PhoXi3DROI &Other) const;
};

#pragma pack(1)
///Defines data cutting according to a specific threshold value.
/**
Defines data cutting according to a specific threshold value of angle (in
degrees) between point normal and specified vector. Specifically useful to cut
out parts of the surface that are too diverted from the Camera, or Projector.
*/
class PHOXI_DLL_API PhoXiNormalAngle {
  public:
    /**
    Maximal angle between point normal and direction to camera.
    Default 90 degrees.
    */
    double MaxCameraAngle;
    /**
    Maximal angle between point normal and direction to projector.
    Default 90 degrees.
    */
    double MaxProjectorAngle;
    /**
    Minimal angle between normal and halfway vector.
    Default 0 degrees
    */
    double MinHalfwayAngle;
    /**
    Maximal angle between normal and halfway vector.
    Default 0 degrees.
    */
    double MaxHalfwayAngle;
    PhoXiNormalAngle();
    PhoXiNormalAngle(const PhoXiNormalAngle &Other);
    PhoXiNormalAngle &operator=(const PhoXiNormalAngle &Other);
    bool operator==(const PhoXiNormalAngle &Other) const;
};

//Forward Declaration for internal purposes
class PhoXiSurfaceSmoothnessInternal;
#pragma pack(1)
///Defines the setting of smoothness of point cloud generation algorithm.
/**
Defines the setting of smoothness of point cloud generation algorithm.
Sharp - optimized for small feature retrieval. Higher noise on surfaces.
Normal - standart sensor setting best for most scanes. Smooth - edge preserving
algorithm that smooths the surface, lowering down noise for expense for small
features. Default value is Normal.
*/
class PHOXI_DLL_API PhoXiSurfaceSmoothness {
  public:
    enum Value {
        NoValue = 0,
        Sharp = 1,
        Normal = 2,
        Smooth = 3
    };
  private:
    friend class PhoXiSurfaceSmoothnessInternal;
    Value SurfaceSmoothness;
  public:
    PhoXiSurfaceSmoothness();
    PhoXiSurfaceSmoothness(const PhoXiSurfaceSmoothness &Other);
    PhoXiSurfaceSmoothness(const Value &SurfaceSmoothness);
    PhoXiSurfaceSmoothness(const int &Other);
    bool operator==(const PhoXiSurfaceSmoothness &Other) const;
    bool operator==(const Value &Other) const;
    bool operator=(const PhoXiSurfaceSmoothness &Other);
    bool operator=(const Value &SurfaceSmoothness);
    bool operator=(const int &SurfaceSmoothness);
    bool operator=(const std::string &SurfaceSmoothness);
    operator Value() const;
    operator std::string() const;
};

#pragma pack(1)
///Capturing Settings are settings that affect the data capturing phase.
/**
By changing the parameters, the time of the scanning can change.
*/
class PHOXI_DLL_API PhoXiCapturingSettings {
  public:
    /**
    Multiplication of the Basic scanner shutter time. With Shutter Multiplier,
    you can control the time of individual patterns of the scanner. Higher
    number of shutter multiplier will help with the darker materials, but will
    increase the scanning time multiplicatively. PhoXi devices uses Scanned
    laser to project patterns, ShutterMultiplier does not affect the time of
    individual sweep, it rather tells the number of repeats of individual
    pattern. It affects the scanning process and the final Scanning Time is
    multiplied by ShutterMultiplier.
    */
    int ShutterMultiplier;
    /**
    Defines the number of scans that will be taken and merged to single output.
    With Scan Multiplier, you can scan multiple times and merge the output to a
    single 3D cloud. This will increase the S/N ratio to ensure higher quality
    scan. It is useful in scenarios with High Dynamic Range, where will increased
    ShutterMultiplier means over saturation of some areas of the scanning. It
    affects the scanning process and the final Scanning Time is multiplied by
    ScanMultiplier.
    */
    int ScanMultiplier;
    /**
    Specify a mode, where only the camera captures the frame. With Camera Only
    Mode, you can use the scanner internal camera to capture only 2D images of
    the scene. This images then can be read out as Texture. It is useful to
    either navigate the scanner around the scene, or to take a quick snapshot
    that can look for changes. It does not perform any computations necessary
    for 3D Scanning, low latency.
    */
    bool CameraOnlyMode;
    /**
    Enables the mode that suppress ambient illumination. Ambient light
    suppression samples the scene multiple times during one pattern exposure.
    This multiplesamples are then used to suppress the effect of ambient
    illumination by eliminating most of the shot noise caused by longer
    exposure of ambient light. Enabling the mode will fix Shutter multiplier
    to the value of 2.
    */
    bool AmbientLightSuppression;
    /**
    Specifies maximum FPS. Useful for limiting the fps in freerun mode. If you
    want to allow a scan every 10 seconds, you can set MaximumFPS to 0.1. If
    you want 2 scans per second, set the value to 2.0. Default is 0 (unlimited).
    */
    double MaximumFPS;
    /**
    Time for projection of one pattern. Single Pattern Exposure is the time for
    projection of one pattern. This value is in milliseconds. Higher value
    slows down the mirror movement, thus pattern is projected slower. Setting
    this value higher may be useful when scanning dark or very shiny objects,
    or when there is another source of strong light and the projected pattern
    might not be easily visible.
    */
    double SinglePatternExposure;
    /**
    Uses strategy optimized for interreflections or normal strategy. Coding
    strategy optimized for Interreflections use advance digital coding to make
    diffuse inter-reflection possible to suppress. For most scenes, strategy
    optimized for Interreflections provides better output, but in some edge cases
    of glossy inter-reflections, Normal could provide more robust results.
    */
    PhoXiCodingStrategy CodingStrategy;
    /**
    Approach to achieve subpixel accuracy. Approach to achieve subpixel
    accuracy. Fast - without subpixel accuracy, High - sub-pixel accuracy,
    Ultra - enhanced sub-pixel accuracy. This parameter influences the
    processing time. Coding quality set to Ultra ensures in many cases best
    accuracy but fewer measured points.
    */
    PhoXiCodingQuality CodingQuality;
    /**
    Defines the used texture source. Defines the used texture source. LED will
    capture an additional image with LED flash that offers a speckle less 2D
    image. Use this setting to achieve nice coloured PointCloud. Computed is
    a texture computed from the structured patterns - no additional image is
    necessary (Lower Quality), Laser will trigger an additional image with
    Laser flash. Use this setting to investigate light conditions in the scene.
    Focus will set a structured pattern as a texture. Useful for analyzing
    problems with signal contrast and the setup of the shutter parameters.
    Color option is only available for Scanners with additional color camera.
    */
    PhoXiTextureSource TextureSource;
    /**
    Intensity of projected pattern. Recommended to decrease only when experiencing
    overexposure.
    */
    int LaserPower;
    /**
    Defines used LED Power, Available from fw version 1.2.21
    */
    int LEDPower;
    /**
    Defines projection offset from the left side, Available from fw version 1.2.21.
    */
    int ProjectionOffsetLeft;
    /**
    Defines projection offset from the right side, Available from fw version 1.2.21.
    */
    int ProjectionOffsetRight;
    PhoXiCapturingSettings();
    PhoXiCapturingSettings(const PhoXiCapturingSettings &Other);
    PhoXiCapturingSettings &operator=(const PhoXiCapturingSettings &Other);
    bool operator==(const PhoXiCapturingSettings &Other) const;
};

#pragma pack(1)
///Settings affecting data processing.
class PHOXI_DLL_API PhoXiProcessingSettings {
  public:
    ///Required measuring confidence in mm.
    double Confidence;
    ///Defines a 3D ROI used for data cutting.
    PhoXi3DROI ROI3D;
    ///Defines limiting angles between point normal and various direction vectors.
    PhoXiNormalAngle NormalAngle;
    ///Defines the smoothness of the point cloud generation algorithm.
    PhoXiSurfaceSmoothness SurfaceSmoothness;
    ///Defines if the point cloud should be cut to recommended scannig range.
    bool CalibrationVolumeOnly;
    /**
    Estimates the radius of normal estimation. Higher values (4 max -> 9x9
    neighborhood) offers smoother normals, while smaller values (1, 2) is good
    for retrieving smaller depth details. Affects normal map generation.
    */
    int NormalsEstimationRadius;
    PhoXiProcessingSettings();
    PhoXiProcessingSettings(const PhoXiProcessingSettings &Other);
    bool operator=(const PhoXiProcessingSettings &Other);
    bool operator==(const PhoXiProcessingSettings &Other) const;
};

#pragma pack(1)
///Settings affecting camera calibration.
class PHOXI_DLL_API PhoXiCalibrationSettings {
  public:
    ///Camera's 3x3 matrix describing the mapping from 3D (world) to 2D (image).
    pho::api::CameraMatrix64f CameraMatrix;
    /**
    Camera's distortion coefficients. It's using OpenCV format:
    (k1, k2, p1, p2[, k3[, k4, k5, k6[, s1, s2, s3, s4[, tx, ty]]]]).
    */
    std::vector<double> DistortionCoefficients;
    ///Camera's focus length.
    double FocusLength;
    ///Width and Height of a pixel (in um).
    PhoXiSize_64f PixelSize;
    PhoXiCalibrationSettings();
    PhoXiCalibrationSettings(const PhoXiCalibrationSettings &Other);
    bool operator=(const PhoXiCalibrationSettings &Other);
    bool operator==(const PhoXiCalibrationSettings &Other) const;
};

//Forward Declaration for internal purposes
class AdditionalCameraCalibrationInternal;
#pragma pack(1)
///Calibration settings for additional camera.
/**
Used as output of calibration of additional camera and input for computation
of aligned depth map.
*/
class PHOXI_DLL_API AdditionalCameraCalibration {
  public:
    ///Settings affecting camera calibration.
    PhoXiCalibrationSettings CalibrationSettings;
    ///Custom coordinates transformation.
    PhoXiCoordinateTransformation CoordinateTransformation;
    ///Resolution of additional camera.
    PhoXiSize CameraResolution;
    AdditionalCameraCalibration();
    AdditionalCameraCalibration(const AdditionalCameraCalibration &Other);
    bool operator=(const AdditionalCameraCalibration &Other);
    bool operator==(const AdditionalCameraCalibration &Other) const;
    bool SaveToFile(const std::string &FileName);
    bool LoadFromFile(const std::string &FileName);
  private:
    std::vector<std::string> SplitString(std::string &Input);
    static inline void Trim(std::string &Input);
    static inline void LeftTrim(std::string &Input);
    static inline void RightTrim(std::string &Input);
    friend class AdditionalCameraCalibrationInternal;
};

//Forward Declaration for internal purposes
class PhoXiTemperaturesReaderInternal;
#pragma pack(1)
///Temperature reader for device sensors.
class PHOXI_DLL_API PhoXiTemperaturesReader {
  public:
    enum Value {
        TempReaderNone = 0,
        TempReaderAll = 1
    };
    PhoXiTemperaturesReader();
    PhoXiTemperaturesReader(const PhoXiTemperaturesReader &Other);
    PhoXiTemperaturesReader(const Value &TemperaturesReader);
    PhoXiTemperaturesReader(const int &TemperaturesReader);
    bool operator=(const PhoXiTemperaturesReader &Other);
    bool operator=(const Value &TemperaturesReader);
    bool operator=(const int &TemperaturesReader);
    bool operator=(const std::string &TemperaturesReader);
    bool operator==(const PhoXiTemperaturesReader &Other) const;
    bool operator==(const Value &TemperaturesReader) const;
    operator Value() const;
    operator std::string() const;
  private:
    Value TemperaturesReader;
};

//Forward Declaration for internal purposes
class PhoXiProfilesDescriptorListInternal;
#pragma pack(1)
///Descriptor of device profiles
class PHOXI_DLL_API PhoXiProfileDescriptor {
  public:
    std::string Name;
    bool IsFactory;
    PhoXiProfileDescriptor();
    PhoXiProfileDescriptor(const PhoXiProfileDescriptor &Other);
    PhoXiProfileDescriptor& operator=(const PhoXiProfileDescriptor &Other);
    bool operator==(const PhoXiProfileDescriptor &Other) const;
};

#pragma pack(1)
/// Descriptor of device profiles
class PHOXI_DLL_API PhoXiProfileContent {
public:
    std::string Name;
    std::vector<unsigned char> Content;
    PhoXiProfileContent();
    PhoXiProfileContent(const PhoXiProfileContent &Other);
    PhoXiProfileContent &operator=(const PhoXiProfileContent &Other);
    bool operator==(const PhoXiProfileContent &Other) const;
};

//Forward Declaration for internal purposes
class PhoXiOperationModeInternal;
#pragma pack(1)
class PHOXI_DLL_API PhoXiOperationMode {
  public:
    enum Value {
        NoValue = 0,
        Camera = 1,
        Scanner = 2,
        Mode2D = 3
    };
    PhoXiOperationMode();
    PhoXiOperationMode(const PhoXiOperationMode &Other);
    PhoXiOperationMode(const Value &OperationMode);
    PhoXiOperationMode(const int &OperationMode);
    PhoXiOperationMode &operator=(const PhoXiOperationMode &Other);
    PhoXiOperationMode &operator=(const Value &OperationMode);
    PhoXiOperationMode &operator=(const int &OperationMode);
    PhoXiOperationMode &operator=(const std::string &OperationMode);
    bool operator==(const PhoXiOperationMode &Other) const;
    bool operator==(const Value &OperationMode) const;
    operator Value() const;
    operator std::string() const;
  private:
    Value OperationMode;
};

//Forward Declaration for internal purposes
class PhoXiMotionCamInternal;
#pragma pack(1)
class PHOXI_DLL_API PhoXiMotionCam {
  public:
    PhoXiOperationMode OperationMode;
    int LaserPower;
    double MaximumFPS;
    PhoXiMotionCam();
    PhoXiMotionCam(const PhoXiMotionCam &Other);
    PhoXiMotionCam &operator=(const PhoXiMotionCam &Other);
    bool operator==(const PhoXiMotionCam &Other) const;
};

//Forward Declaration for internal purposes
class PhoXiSamplingTopologyInternal;
#pragma pack(1)
class PHOXI_DLL_API PhoXiSamplingTopology {
  public:
    enum Value {
        NoValue = 0,
        Standard = 1
    };
    PhoXiSamplingTopology();
    PhoXiSamplingTopology(const PhoXiSamplingTopology &Other);
    PhoXiSamplingTopology(const Value &SamplingTopology);
    PhoXiSamplingTopology(const int &SamplingTopology);
    PhoXiSamplingTopology &operator=(const PhoXiSamplingTopology &Other);
    PhoXiSamplingTopology &operator=(const Value &SamplingTopology);
    PhoXiSamplingTopology &operator=(const int &SamplingTopology);
    PhoXiSamplingTopology &operator=(const std::string &SamplingTopology);
    bool operator==(const PhoXiSamplingTopology &Other) const;
    bool operator==(const Value &SamplingTopology) const;
    operator Value() const;
    operator std::string() const;
  private:
    Value SamplingTopology;
};

//Forward Declaration for internal purposes
class PhoXiOutputTopologyInternal;
#pragma pack(1)
class PHOXI_DLL_API PhoXiOutputTopology {
  public:
    enum Value {
        NoValue = 0,
        IrregularGrid = 1,
        Raw = 2,
        RegularGrid = 3
    };
    PhoXiOutputTopology();
    PhoXiOutputTopology(const PhoXiOutputTopology &Other);
    PhoXiOutputTopology(const Value &OutputTopology);
    PhoXiOutputTopology(const int &OutputTopology);
    PhoXiOutputTopology &operator=(const PhoXiOutputTopology &Other);
    PhoXiOutputTopology &operator=(const Value &OutputTopology);
    PhoXiOutputTopology &operator=(const int &OutputTopology);
    PhoXiOutputTopology &operator=(const std::string &OutputTopology);
    bool operator==(const PhoXiOutputTopology &Other) const;
    bool operator==(const Value &OutputTopology) const;
    operator Value() const;
    operator std::string() const;
  private:
    Value OutputTopology;
};

//Forward Declaration for internal purposes
class PhoXiMotionCamCameraModeInternal;
#pragma pack(1)
class PHOXI_DLL_API PhoXiMotionCamCameraMode {
  public:
    double Exposure;
    PhoXiSamplingTopology SamplingTopology;
    PhoXiOutputTopology OutputTopology;
    PhoXiCodingStrategy CodingStrategy;
    PhoXiMotionCamCameraMode();
    PhoXiMotionCamCameraMode(const PhoXiMotionCamCameraMode &Other);
    PhoXiMotionCamCameraMode &operator=(const PhoXiMotionCamCameraMode &Other);
    bool operator==(const PhoXiMotionCamCameraMode &Other) const;
};

//Forward Declaration for internal purposes
class PhoXiMotionCamScannerModeInternal;
#pragma pack(1)
class PHOXI_DLL_API PhoXiMotionCamScannerMode {
  public:
    int ShutterMultiplier;
    int ScanMultiplier;
    PhoXiCodingStrategy CodingStrategy;
    PhoXiCodingQuality CodingQuality;
    PhoXiTextureSource TextureSource;
    PhoXiMotionCamScannerMode();
    PhoXiMotionCamScannerMode(const PhoXiMotionCamScannerMode &Other);
    PhoXiMotionCamScannerMode &operator=(const PhoXiMotionCamScannerMode &Other);
    bool operator==(const PhoXiMotionCamScannerMode &Other) const;
};

#pragma pack(pop)

} // namespace api
} // namespace pho

#endif //_PHOXI_FEATURE_TYPES_H
