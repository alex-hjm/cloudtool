#pragma once
#ifndef _PHOXI_H
#define _PHOXI_H
#include "PhoXiCompilerDefines.h"

/*
Version 1.0 - ReleaseVersion

Photoneo
www.photoneo.com

Parts marked as #ifdef PHOXI_API_SERVER are not relevant for the user

*/

#include "PhoXiInterface.h"

namespace pho {
#ifdef PHOXI_API_SERVER
class ClientSideApp;
class Camera3DApp;
#endif

namespace api {
#if _DEBUG
#  ifdef PHOXI_API_SERVER
#    define PHOXI_OPERATION_TIMEOUT_DEFAULT 10000
#  else
#    define PHOXI_OPERATION_TIMEOUT_DEFAULT OperationTimeout::Infinity
#  endif
#else
#  define PHOXI_OPERATION_TIMEOUT_DEFAULT 10000
#endif

#pragma pack(push, 8)
///Defines the type of the PhoXi Device.
/**
Smart Enumerator of the PhoXi Device type, new supported devices
will beincluded in this enumeration.
*/
class PHOXI_DLL_API PhoXiDeviceType {
  public:
    enum Value {
        NoValue = 0,
        PhoXiScanner,
        MotionCam3D,
        DeviceTypeCount // must remain last
    };
  private:
    Value Type;
  public:
    PhoXiDeviceType();
    PhoXiDeviceType(const Value Type);
    PhoXiDeviceType(const int Type);
    PhoXiDeviceType(const std::string &Type);
    PhoXiDeviceType &operator=(const std::string &Type);
    operator Value() const;
    operator std::string() const;
};
#pragma pack(pop)

//Forward declaration
class PhoXi;

#pragma pack(push, 8)
///Main handle type for the PhoXi Device.
/**
PPhoXi is an extended shared pointer, it does automatically free all
resources after last reference goes out of scope.
*/
class PHOXI_DLL_API PPhoXi {
  private:
    std::shared_ptr <PhoXi> PhoXiPtr;
  public:
    PhoXi *operator->() const;
    PPhoXi(const PPhoXi &Other);
    PPhoXi(const std::shared_ptr <PhoXi> &Other);
    PPhoXi(std::nullptr_t);
    PPhoXi();
    PPhoXi &operator=(const PPhoXi &Other);
    PPhoXi &operator=(const std::shared_ptr <PhoXi> &Other);
    operator std::shared_ptr<PhoXi>() const;
    operator std::weak_ptr<PhoXi>() const;
    operator bool() const;
    ///Free the pointed resource and set the Pointer to nullptr.
    void Reset();
};
#pragma pack(pop)

//Forward declaration
class PhoXiCommunicationTools;
//Forward declaration
class PhoXiRawAccessHandler;
struct PhoXiDeviceInformation;

#pragma pack(push, 8)
///PhoXi is the handler of the PhoXi Device - The instance of PPhoXi is returned by PhoXiFactory.
/**
Main object for the communication with the device.
The object consists of multiple features, that controls the device configuration.
    These features uses Setter and Getter functions in background to communicate
    directly with the device. If the Feature value is changed by assign, it will
    automatically apply the value to the device. Some features could not be
    available or can be read only or write only. Look for documentation of
    PhoXiFeature for more informations.
    Ask for Feature.isLastOperationSuccessful() to check if the last set or get
    of the feature was successful. Ask for Feature.GetLastErrorMessage() to
    check the error.
PhoXi is not designed as Thread-Safe, it is intended to be used and operated in
    single thread. You are allowed to capture frames from the separate thread as
    the settings change.
*/
class PHOXI_DLL_API PhoXi : public PhoXiInterface {
    friend class PhoXiFactory;
  public:
    PhoXi();
    virtual ~PhoXi();
    ///Returns the type of the device.
    PhoXiDeviceType GetType() const;
    ///Ask if the device is connected to the Hardware.
    /**
    When the device is returned by PhoXiFactory by Create command, it is not
    connected to physical Hardware. To connect the device, call Connect command
    of the specified Handle. It does connect to specific device based on
    HWIdentification.
    @return true if connected to the device.
    */
    bool CanConnect();
    ///Make a connection to the physical Device trough PhoXi Control.
    /**
    Make a connection to the physical device trough the PhoXi Control.
    If the device is not connected to the PhoXi Control, it will establish
    the communication and connect. If the device is connected to PhoXi Control,
    it will connect immediately. Multiple Processes can connect to a single
    PhoXi device and receive Frames from cyclic buffer. The pop-ing of the
    Frames from one process does not affect the other process.
    @return true on success.
    */
    virtual bool Connect() final;
    ///Terminate the connection with PhoXi Control.
    /**
    Terminate the connection with PhoXi Control.
    If the LogOutDevice param is true, the device will be logged out from PhoXi
    Control and will be available to other PCs on network. If the LogOutDevice
    param is false (default), the device will remain connected to PhoXi Control
    and will be occupied for other PCs on the network. With this option, the
    next reconnection is fast and maintain un-stored settings.
    @param LogOutDevice If true, the device will be logged out from PhoXi
    Control and will be available to other PCs on network.
    @param StopDeviceAcquisition If false, the device will be left in a state
    before the call.
    @return true on success.
    */
    virtual bool Disconnect(bool LogOutDevice = false, bool StopDeviceAcquisition = true) final;
    ///Waits Timeout ms for a new frame.
    /**
    Wait a specified time for a new Frame to arrive, this call does not trigger
    scanning, it just retrieve the Frame from the cyclic buffer, or wait for
    the frame to arrive. To trigger the scan, call either call SoftwareTrigger
    if in Software Trigger mode, or just Wait for frame if in Freerun (in both
    cases, the device has to be in Acquisition mode). This call pop the Frame
    from the cyclic buffer, so calling this call repeatably will empty the buffer.
    If the Frame is already in the buffer, the call of this function with zero
    timeout will return the Frame. If the Timeout pass, the call returns with
    empty PFrame equal to std::nullptr. If the Frame fails, the nonempty structure
    will be returned. Frame->Successful defines if the Captured frame was
    successfully captured.
    @param Timeout - Timeout in ms, or PhoXiTimeout::ZeroTimeout,
    PhoXiTimeout::Infinity, PhoXiTimeout::LastStored
    Default is PhoXiTimeout::LastStored - Can be set by changing Timeout feature.
    @return nonempty PFrame on success, nullptr otherwise
    */
    virtual PFrame GetFrame(PhoXiTimeout Timeout = PhoXiTimeout::LastStored) override final;
    ///Waits Timeout ms for a new frame.
    /**
    Wait a specified time for a new Frame to arrive, this call does not trigger
    scanning, it just retrieve the Frame from the cyclic buffer, or wait for
    the frame to arrive. To trigger the scan, call either call SoftwareTrigger
    if in Software Trigger mode, or just Wait for frame if in Freerun (in both
    cases, the device has to be in Acquisition mode). This call pop the Frame
    from the cyclic buffer, so calling this call repeatably will empty the
    buffer. If the Frame is already in the buffer, the call of this function
    with zero timeout will return the Frame. If the Timeout pass, the call
    returns with empty PFrame equal to std::nullptr. If the Frame fails, the
    nonempty structure will be returned. Frame->Successful defines if the
    Captured frame was successfully captured.
    @param Timeout - Timeout in ms, or PhoXiTimeout::ZeroTimeout,
    PhoXiTimeout::Infinity, PhoXiTimeout::LastStored.
    @param RealDuration - Output Real duration of the call.
    @param PrintError - Specify if grabbing failures are written into console.
    @return nonempty PFrame on success, nullptr otherwise.
    */
    virtual PFrame GetFrame(PhoXiTimeout Timeout, double &RealDuration, bool PrintError = true) override final;
    ///Waits Timeout ms for a frame with a specific FrameID, other frames are scratched.
    /**
    Wait a specified time for a new Frame to arrive with a specific FrameID,
    this call does not trigger scanning, it just retrieve the Frame from the
    cyclic buffer, or wait for the frame to arrive. To trigger the scan, call
    either call SoftwareTrigger if in Software Trigger mode, or just Wait for
    frame if in Freerun (in both cases, the device has to be in Acquisition
    mode). This call pops the Frame from the cyclic buffer, on success, it will
    compare the FrameID with the FrameID of the Frame. If these are equal, the
    call will return the frame, otherwise, it will scratch the frame and repeat
    until timeout. If the Frame with the specific FrameID is already in the
    buffer, the call of this function with zero timeout will return the Frame.
    If negative value is supplied, it will return nullptr. If the Timeout pass,
    the call returns with empty PFrame equal to std::nullptr. If the Frame
    fails, the nonempty structure will be returned. Frame->Successful defines
    if the Captured frame was successfuly captured.
    @param Timeout - Timeout in ms, or PhoXiTimeout::ZeroTimeout,
    PhoXiTimeout::Infinity, PhoXiTimeout::LastStored. Default is
    PhoXiTimeout::LastStored - Can be specified by setting Timeout feature.
    @return nonempty PFrame on success, nullptr otherwise.
    */
    virtual PFrame GetSpecificFrame(int FrameID, PhoXiTimeout Timeout = PhoXiTimeout::LastStored) override final;
    ///Enable asynchronous notifications with arriving frames
    /**
    Method EnableAsyncGetFrame set callback called when new frame from the
    device arrived to the buffer. Callback execution is on dispatched thread.
    Custom dispatching thread can be set via SetSynchronizationContext function
    from factory. EnableAsyncGetFrame method is threadsafe and new callback can
    be set during acquisition, but it's not recommended, because old callback
    functor can be already scheduled in queue and new frame is notified through
    it. Asynchronous frame getter is exclusive, GetFrame and GetSpecificFrame
    are not effective when mode is enabled.
    @param NewFrameArrivedCallback Callback functor executed on dispatched thread.
    @return true on success, false otherwise.
    */
    virtual bool EnableAsyncGetFrame(std::function<void(PFrame)>&& NewFrameArrivedCallback) override final;
    ///Disable asynchronous notifications with arriving frames
    /**
    Method DisableAsyncGetFrame give control for getting frames back to
    GetFrame and GetSpecificFrame.
    @return true on success, false otherwise.
    */
    virtual bool DisableAsyncGetFrame() override final;
    ///Clear the Frame buffer, returns cleared number of frames.
    int ClearBuffer();
    ///Go to acquisition mode.
    /**
    Start acquisition mode. After success, the device will be ready to capture
    3D Frames. If in Freerun, the device will trigger itself in maximal
    possible Frame rate, or MaximalFPS defined by user. If in Software trigger
    mode, the device will Wait for TriggerImage calls.
    @return true on success, false otherwise.
    */
    virtual bool StartAcquisition() override;
    ///Stops the acquisition mode.
    /**
    Stop acquisition mode. After success, the device will stop capturing 3D
    Frames, or react to Software trigger. If in Freerun, the device will stop
    the scanning after the last Frame is captured. If in Software trigger mode,
    the device will not respond to TriggerFrame calls.
    @return true on success, false otherwise.
    */
    virtual bool StopAcquisition() override;
    ///Triggers the Frame.
    /**
    Triggers the Frame (if in Software Trigger mode) and returns FrameID on success
    The FrameID can be used to pair the Frames received by GetFrame with the Software Triggers
    If the device is not in Software trigger mode, the call will return negative number
    If WaitForAccept is true, the call will wait until the device will be ready to accept the Software trigger
    If WaitForAccept if false, the call will return with negative number after the unsuccessful Software trigger attempt (i.e. multiple Rapid Software triggers)
    If WaitForGrabbingEnd is true, the call will wait until the end of Device Grabbing and then returns - This is useful for Data Grabbing Sync between more devices
        Returns -4 if not supported
    @param WaitForAccept - Wait for the device until ready for Software Trigger
    @param CustomMessage - Custom string that will be transfered and attached to the frame (Can be used to transfer robotic coordinates ...) - PhoXiControl of 1.2 > is needed
    @return positive FrameID on success, negative number on failure (-1 Trigger not accepted, -2 Device is not running, -3 Communication Error, -4 WaitForGrabbingEnd is not supported, -5 Timeout)
    */
    virtual int TriggerFrame(bool WaitForAccept = true, bool WaitForGrabbingEnd = false, const std::string& CustomMessage = "") override;
    ///Saves last output DM to specific file format and return true on success.
    /**
    Saves last output scan to specific file format and return true on success.
    Supported file formats are:
    Text file (*.txt),
    Stanford's PLY (*.ply),
    Leica's PTX (*.ptx),
    Photoneo's Raw data format (*.praw),
    Photoneo's Raw data format - Expanded to folder structure (*.prawf),
    Raw images data format in tif (*.tif),
    @param FilePath - Path where the file should be saved. Absolute path is
    preferred (e.g. C:/Users/Public/*.praw, /home/user/*praw), relative path
    points to PhoXiControl's path (folder should exist).
    @param FrameID - Save only if last output scan has the same frame index.
    @return true on success, false otherwise.
    */
    virtual bool SaveLastOutput(const std::string &FilePath, int FrameID = -1) override;
    ///Lock the GUI controls for this device.
    bool LockGUI();
    ///Unlock the GUI controls for this device.
    bool UnlockGUI();
    ///Save current device settings to the device internal memory.
    virtual bool SaveSettings() override final;
    ///Reset active preset so that settings from internal memory of device will be loaded.
    virtual bool ResetActivePreset() override;
#ifdef PHOXI_API_SERVER
    friend class Camera3DApp;
    Camera3DApp* Owner;
    void SetOwner(Camera3DApp* Owner);
    bool AddFrame(const Frame& NewFrame);
#endif
    std::unique_ptr <PhoXiCommunicationTools> CommunicationTools;

  protected:
    virtual bool ConnectPhoXi();
    virtual bool DisconnectPhoXi(bool LogOutDevice = false);

    virtual bool SetResolution(const PhoXiSize &Resolution, PhoXiFeature<PhoXiSize> &Feature) override;
    virtual PhoXiSize GetResolution(PhoXiFeature<PhoXiSize> &Feature) override;

    virtual bool SetTimeout(const PhoXiTimeout &Timeout, PhoXiFeature<PhoXiTimeout> &Feature) override;
    virtual PhoXiTimeout GetTimeout(PhoXiFeature<PhoXiTimeout> &Feature) override;
    friend class PhoXiCommunicationTools;
    friend class PhoXiRawAccessHandler;
    virtual int TriggerDeviceFrame(bool WaitForAccept = true, bool WaitForGrabbingEnd = false, const std::string& CustomMessage = "") override;
    virtual bool SaveLastDeviceOutput(const std::string &FilePath, int FrameID = -1) override;
    virtual bool CheckConnectionStatus() override;

    virtual bool GetFrame(PhoXiTimeout Timeout, PFrame& Frame);
    virtual bool GetFrame(PhoXiTimeout Timeout, double &RealDuration, bool PrintError, PFrame& Frame);
    virtual bool GetSpecificFrame(int FrameID, PhoXiTimeout Timeout, PFrame& Frame);

    bool GetHWStatus(PhoXiStatus &Status, PhoXiTriggerMode &TriggerMode);
    bool SyncHWStatus();
    virtual bool StartPhoXiAcquisition() override;
    virtual bool StopPhoXiAcquisition() override;
    virtual bool LogOut() override;
    virtual bool ConnectIPv4(const std::string &IPv4) final;
    void UpdateFeatures();
    bool UpdateDeviceInfo();

    std::unique_ptr<PhoXiDeviceInformation> deviceInformation;
    PhoXiDeviceType type;

  private:
    int64_t _FrameIdRead = -1;
    void UpdateFrameIdRead(int64_t frameId);
    bool FrameIdWillBeAvailable(int64_t frameId);
};
#pragma pack(pop)

#pragma pack(push, 8)
///Information about the connected PhoXi device.
struct PHOXI_DLL_API PhoXiDeviceInformation {
    std::string Name;
    PhoXiDeviceType Type;
    std::string HWIdentification;
    PhoXiConnectionStatus Status;
    std::string FirmwareVersion;
    std::string Variant;
    ///File camera is loaded source from PRAW, not connect to real/emulated scanner.
    bool IsFileCamera;
    ///Identify if the device is connected to PhoXi Control.
    bool ConnectedToPhoXiControl() const;
    PhoXiDeviceInformation();
    std::string GetTypeHWIdentification() const;
    operator std::string() const;
};
#pragma pack(pop)

//Forward declaration
class PhoXiFactoryTools;

/// PhoXi custom implementation of synchronisation context for asynchronous new frame arrived callback.
/**
Derive you own class from PhoXiSynchronizationContext and then you can process asynchronous callback
in own thread.
*/
class PHOXI_DLL_API PhoXiSynchronizationContext
{
public:
    virtual ~PhoXiSynchronizationContext() = default;

    /// Initialize context, called from pho api, guarantee one call.
    /**
    Pho framework call Init method only once before any other calls.
    */
    virtual void Init() = 0;
    /// Deinitialize context, called from pho api, guarantee one call.
    /**
    Pho framework call Deinit method only once, no other calls
    after Deinit are not executed.
    */
    virtual void Deinit() = 0;

    // Execute task asynchronously.
    /**
    @param tag Diagnostics tag as an identifier who created task
    @param task Simple functor which can be executed in your own thread.
    If task is executed on same thread as Post method is called,
    then there is a risk of critical thread suspension.
    */
    virtual void Post(const char* tag, std::function<void()>&& task) = 0;
};

#pragma pack(push, 8)

///PhoXi Factory is the main Class for dynamic creation of PhoXi Objects.
/**
PhoXi Factory object can be used locally, it does provide high-level
communication with the PhoXi Control. It is based on singleton pattern, so
multiple Factories will use the same communication channel. The communication
is dropped as soon as the last PhoXiFactory object goes out of scope. The
communication channel used by PhoXiFactory is separate from the channel used
for any connected device.
PhoXi Factory is designed as Thread-Safe.
*/
class PHOXI_DLL_API PhoXiFactory {
  public:
    ///Creates new PhoXi object of a specified Type.
    /**
    Creates new PhoXi object of a specified Type (object Factory pattern)
    The object targets to a non-connected device, To connect the device, set
    correct HWIndentification and call Connect command.
    @param Type - PhoXiDeviceType enumerator - device type.
    @return handle to a !non-connected device if correct type is supplied,
    returns nullptr otherwise.
    */
    PPhoXi Create(PhoXiDeviceType Type);
    ///Creates new PhoXi object.
    /**
    Creates new PhoXi object of a specified "Type" or "[Type]-(HWIdentification)"
    as an argument. The latter can be taken from GetDeviceList. The object
    targets to a non-connected device, To connect the device, set correct
    HWIndentification (if not supplied in the argument) and call Connect command.
    @param Type - TypeName - device type - can be obtained from the specified
    PhoXiDeviceType by string conversion.
    @return handle to a !non-connected device if correct type is supplied,
    returns nullptr otherwise.
    */
    PPhoXi Create(const std::string &Type);
    ///Creates new PhoXi object.
    /**
    Creates new PhoXi object of a specified PhoXiDeviceInformation object
    as an argument. The object targets to a non-connected device, to connect
    the device and call Connect command.
    @param DeviceInfo - Can be obtained from the specified GetDeviceList.
    @return handle to a !non-connected device if correct type is supplied,
    returns nullptr otherwise.
    */
    PPhoXi Create(const PhoXiDeviceInformation &DeviceInfo);
    ///Creates new PhoXi object of a specified Type.
    /**
    Creates new PhoXi object of a specified Type (object Factory pattern).
    The object targets to a non-connected device, To connect the device,
    set correct HWIndentification and call Connect command.
    @param Type - PhoXiDeviceType enumerator - device type.
    @return handle to a !non-connected device if correct type is supplied,
    returns nullptr otherwise.
    */
    PPhoXi Create(int Type);
    ///Creates new PhoXi object of a specified Type.
    /**
    Creates new PhoXi object (object Factory pattern).
    The object targets to a non-connected device, To connect the device,
    set correct HWIndentification and call Connect command.
    @return handle to a !non-connected device
    */
    PPhoXi Create();

#ifdef PHOXI_API_CLIENT
    ///Creates PhoXi object based on HWIdentification and Connects.
    /**
    The call creates an object of a correct type based on physical device with
    specific HWIdentification. After that, the handle is connected to the
    physical device. The call repetitively try to Create and Connect the device
    for a specified time (if zero is supplied, only single try will be
    performed). The call can be used even when the device is detached from the
    network, As soon as the device will be available the call will continue
    and return the handle.
    @param HWIdentification The input Hardware identification string,
    or ID (same as in PhoXi Control) of the device.
    @param Timeout Timeout in ms to wait for the device. If the device is not
    currently available, it will wait until the device will be ready, or will
    timeout.
    @return handle to a connected PhoXi device or nullptr on failure
    */
    PPhoXi CreateAndConnect(const std::string& HWIdentification, PhoXiTimeout Timeout = PhoXiTimeout::Infinity);
    ///Create PhoXi object based on HWIdentification, DeviceType and IP and Connects.
    /**
    The call creates an object of a supplied DeviceType type with and connect
    to device with supplied HWIdentification and specified IP address. After
    that, the handle is connected to the physical device. This call can be
    used, if the IP address of the device is known and the device is not
    present in the Network discovery of PhoXi Control. In case that the device
    is visible, it will perform the same operation as:
    CreateAndConnect(HWIdentification, PhoXiTimeout::Zero), ignoring IP address.
    @param HWIdentification The input Hardware identification string, or ID
    (same as in PhoXi Control) of the device.
    @param Type - PhoXiDeviceType enumerator - device type.
    @param IP4 - IP of the device.
    @return handle to a connected PhoXi device or nullptr on failure.
    */
    PPhoXi CreateAndConnect(const std::string& HWIdentification, PhoXiDeviceType Type, const std::string& IP4);
    ///Creates PhoXi object based on the first connected device and connects.
    /**
    The call creates an object of a correct type based on the first device
    connected to PhoXi Control and connects to it immediately. After that,
    the handle is connected to the physical device.
    @return handle to a connected PhoXi device or nullptr if no device is
    connected to PhoXi Control or on failure.
    */
    PPhoXi CreateAndConnectFirstAttached();
    ///Givess the list of all PhoXi devices on network.
    /**
    The list of PhoXi devices available on local network.
    Specific PhoXi instances can be initiated using Create method.

    @param RefreshDeviceDiscovery Restart looking for devices - default
    is false. It may lasts longer, but may retrieve from inconsistent
    network status.
    @return vector filled with PhoXiDeviceInformation on success,
    empty vector otherwise.
    */
    std::vector <PhoXiDeviceInformation> GetDeviceList(bool RefreshDeviceDiscovery = false);
    ///Query the communication status with the PhoXi Control.
    /**
    It does ping the communication with PhoXi Control. If the app is not
    running, or if there is a problem with the connection, the call will
    return false.
    @return true if connection is alive with running PhoXi Control,
    returns false otherwise.
    */
    bool isPhoXiControlRunning();
    ///Query the version of connected PhoXi Control.
    /**
    It does retrieve the version of connected PhoXi Control, the version
    defines capability of PhoXi Control. For best results, try to keep the
    API version the same as PhoXi Control Version.
    @return Version string on success, empty string otherwise.
    */
    std::string GetPhoXiControlVersion();
    ///Query the version of current API
    /**
    It does retrieve the version of current API, the version defines capability
    of API. For best results, try to keep the API version the same as PhoXi
    Control Version.
    @return Version string on success, empty string otherwise.
    */
    std::string GetAPIVersion();
    ///Minimizes PhoXi Control GUI to save up processing power.
    bool MinimizePhoXiControl();

    ///Attaches File 3D Camera to PhoXi Control Network Discovery.
    /**
    It does attach a File 3D Camera with the specified praw files. If folder
    is passed as one of FilePaths entries, it will recursively add all internal
    praw files into the queue.
    @FileCameraHWIdentification defines the desired name for the File 3D Camera
    Device. The call itself will add a suffix to the name.
    @FilePaths are paths to the individual praws or folders containing praws.
    @return Name of the added Camera or empty string on failure.
    */
    std::string AttachFileCamera(const std::string& FileCameraHWIdentification, const std::vector<std::string>& FilePaths, bool MultiByteString=false);
    ///Detaches File 3D Camera from PhoXi Control Network Discovery.
    /**
    It does detach the specified File 3D Camera from the Network Discovery list.
    @FileCameraHWIdentification defines the desired name of the File 3D Camera
    that has to be detached.
    @return true on success of false on failure.
    */
    bool DetachFileCamera(const std::string& FileCameraHWIdentification);
    /// Set user specific synchronisation context.
    /**
    It does set custon user synchronisation context used for asynchronous new
    frame arrived callback set by EnableAsyncGetFrame method.
    If no custom synchronisation context is not set, then default is using.
    Lifetime of input pointer is not managed by pho api. If context is no
    longer used, then Deinit method is called, so in method Deinit context
    itself can be released.
    Context can be set only if no device has been connected. Method is not
    thread safe, we recommend to set custom synchronisation context only once,
    immediatelly to factory creation. Creation of more factories or release
    factory has no impact to set context.
    Method returns false only when:
    - ctx is nullptr
    - almost one device is connected
    @param ctx Synchronisation context to be set
    @return true if new context has been set, otherwise false
    */
    bool SetSynchronizationContext(pho::api::PhoXiSynchronizationContext* ctx);
#endif

    ///Constructor
    PhoXiFactory();
    ///Returns true, it is possible to connect the device with specified HWIdentification.
    /**
    It is not allowed for a single application to connect two separate handles
    to a single device the call for CanConnect can tell if such device is
    already connected to some handle.
    @return true on success, false otherwise.
    */
    bool CanConnect(std::string HardwareIdentification);
  private:
    friend class PhoXi;
    friend class ClientSideApp;
    bool ConnectPhoXi(const std::string &PhoXiType, const std::string &HWIdentification);
    bool ConnectPhoXi(const std::string &PhoXiType, const std::string &HWIdentification, const std::string &IPv4, const std::string &Mode);
    std::shared_ptr <PhoXiFactoryTools> FactoryTools;
  public:
    bool RemovePhoXi(const std::string &HWIdentification);
    //For Debugging purposes only
    void StartConsoleOutput(const std::string &Type);
    bool isInDebugMode();
#ifdef PHOXI_API_SERVER
    void Initiate(const std::shared_ptr<pho::ClientSideApp>& clientSideApp);
    //Creating an physical object of PhoXi Factory will ensure the application will Initialize and de-initialize on destruction even in case of exception
    PhoXiFactory(const std::shared_ptr<pho::ClientSideApp>& clientSideApp);
#endif
  public:
};
#pragma pack(pop)
}
}

#endif //_PHOXI_H

