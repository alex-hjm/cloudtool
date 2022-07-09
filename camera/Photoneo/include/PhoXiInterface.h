#pragma once
#ifndef _PHOXI_INTERFACE_H
#define _PHOXI_INTERFACE_H
#include "PhoXiCompilerDefines.h"


#include <memory>
#include "PhoXiFeatureTypes.h"
#include "PhoXiFeature.h"
#include "PhoXiFeatureList.h"

namespace pho {
namespace api {

#pragma region FEATURE_MACROS
#define PHOXI_FEATURE_SET_ERROR(ERROR_MESSAGE) SetFeatureErrorMessage(Feature, ERROR_MESSAGE)
#define PHOXI_FEATURE_SET_VALUE(VALUE) SetFeatureValue(Feature, VALUE)

#define PHOXI_DECLARE_FEATURE(FEATURE_NAME, TYPE)\
    public:\
    PhoXiFeature<TYPE> FEATURE_NAME;

#define PHOXI_DECLARE_FEATURE_FUNCTIONS(FEATURE_NAME, TYPE)\
    protected:\
        virtual bool Set##FEATURE_NAME(const TYPE& FEATURE_NAME, PhoXiFeature<TYPE>& Feature);\
        virtual TYPE Get##FEATURE_NAME(PhoXiFeature<TYPE>& Feature);

#define PHOXI_BIND_FUNC(FEATURE_NAME, FEATURE_TYPE)\
    FEATURE_NAME.BindFunctions(&PhoXiInterface::Get##FEATURE_NAME, &PhoXiInterface::Set##FEATURE_NAME, this);\
    FEATURE_NAME.SetName(#FEATURE_NAME);FEATURE_NAME.SetOwner(this);\
    FEATURE_NAME.SetTypeName(#FEATURE_TYPE);\
    Features.Add(FEATURE_NAME);

#pragma endregion 

class PHOXI_DLL_API PhoXiInterface {
  public:
    class PHOXI_DLL_API FeatureDatabase {
      private:
        std::vector <std::string> FeatureNames;
        std::vector<PhoXiFeatureInterface *> Features;
        void Add(PhoXiFeatureInterface &Feature);
        PhoXiFeatureInterface *GetFeatureInterface(const std::string &FeatureName);
      public:
        ///Get the list of supported features.
        std::vector <std::string> GetSupportedFeatures() const;
        ///Is feature with a specific name supported?
        bool isFeatureSupported(const std::string &FeatureName) const;
        ///Are specified list of features supported?
        bool areFeaturesSupported(const std::vector <std::string> &FeatureNames) const;
        friend class PhoXiInterface;
    };
    PhoXiInterface();
    virtual ~PhoXiInterface(void);
    ///Ask if the device is connected to the Hardware.
    /**
    When the device is returned by PhoXiFactory by Create command, it is not
    connected to physical Hardware. To connect the device, call Connect command
    of the specified Handle. It does connect to specific device based on
    HWIdentification.
    @return true if connected to the device.
    */
    bool isConnected();
    ///Ask if the device is in Acquisition mode.
    /**
    The connected device could be either in Acquisition mode or not.
    If the device is in Acquisition mode, it is ready to grab images.
    Settings like TriggerMode(Software or Freerun) can be changed only
    when the device is not in acquisition mode.
    There is no connected device for the process with the same
    HardwareIdentification.
    @return true if the device is connected and is in acquisition mode.
    */
    virtual bool isAcquiring();
    ///Set user specified name for the device handle.
    /**
    Set the local name for the device handle
    This can be used for easier description of the device in user code. This
    setting is local, and does not affect the device or PhoXiControl in any way.
    @param Name New name for the device.
    */
    void SetName(const std::string &Name);
    ///Get user specified name for the device handle.
    /**
    Get the local name for the device handle. This can be used for easier
    description of the device in user code. This setting is local, and does
    not affect the device or PhoXiControl in any way.
    @return the user specified name.
    */
    std::string GetName() const;
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
    virtual PFrame GetFrame(PhoXiTimeout Timeout = PhoXiTimeout::LastStored) = 0;
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
    virtual PFrame GetFrame(PhoXiTimeout Timeout, double &RealDuration, bool PrintError = true) = 0;
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
    virtual PFrame GetSpecificFrame(int FrameID, PhoXiTimeout Timeout = PhoXiTimeout::LastStored) = 0;
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
    virtual bool EnableAsyncGetFrame(std::function<void(PFrame)>&& NewFrameArrivedCallback) = 0;
    ///Disable asynchronous notifications with arriving frames
    /**
    Method DisableAsyncGetFrame give control for getting frames back to
    GetFrame and GetSpecificFrame. 
    @return true on success, false otherwise.
    */
    virtual bool DisableAsyncGetFrame() = 0;
    ///Deprecated - use TriggerFrame instead.
    DEPRECATED virtual int TriggerImage(bool WaitForAccept = true);
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
    virtual int TriggerFrame(bool WaitForAccept = true, bool WaitForGrabbingEnd = false, const std::string& CustomMessage = "");
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
    virtual bool SaveLastOutput(const std::string &FilePath, int FrameID = -1);
    ///Go to acquisition mode.
    /**
    Start acquisition mode. After success, the device will be ready to capture
    3D Frames. If in Freerun, the device will trigger itself in maximal
    possible Frame rate, or MaximalFPS defined by user. If in Software trigger
    mode, the device will Wait for TriggerImage calls.
    @return true on success, false otherwise.
    */
    virtual bool StartAcquisition();
    ///Stops the acquisition mode.
    /**
    Stop acquisition mode. After success, the device will stop capturing 3D
    Frames, or react to Software trigger. If in Freerun, the device will stop
    the scanning after the last Frame is captured. If in Software trigger mode,
    the device will not respond to TriggerFrame calls.
    @return true on success, false otherwise.
    */
    virtual bool StopAcquisition();
    ///Save current device settings to the device internal memory.
    virtual bool SaveSettings();
    ///Reset active preset so that settings from internal memory of device will be loaded.
    virtual bool ResetActivePreset();

#define PHOXI_FEATURE_DECLARATION
#include "PhoXiFeatureList.h"
#undef PHOXI_FEATURE_DECLARATION

    FeatureDatabase Features;
  private:
    PhoXiInterface(const PhoXiInterface &Other);
    PhoXiInterface &operator=(const PhoXiInterface &Value);
  protected:
    friend class PhoXiFeatureInterface;
    PhoXiStatus Status;
    std::string Name, Mode;
    template<typename T>
    void EnableFeature(PhoXiFeature<T> &Feature) {
        Feature.Enable();
    }
    void ErrorMessage(const std::string &Message) const;
    template<typename T>
    void EnableFeature(PhoXiFeature<T> &Feature, bool CanGet, bool CanSet, bool FullAccess = false) {
        if (CanGet && CanSet) {
            EnableFeature(Feature, CanGet, CanSet, FullAccess, true);
        } else {
            EnableFeature(Feature, CanGet, CanSet, FullAccess, false);
        }
    }
    template<typename T>
    void EnableFeature(PhoXiFeature<T> &Feature, bool CanGet, bool CanSet, bool FullAccess, bool StoreInFile) {
        Feature.Enable(CanGet, CanSet);
        if (FullAccess) Feature.DisableDefaultControl();
    }
    template<typename T>
    void SetFeatureAccessibility(PhoXiFeature<T> &Feature, bool CanGet, bool CanSet, bool FullAccess = false) {
        Feature.SetAccessibility(CanGet, CanSet);
        if (FullAccess) Feature.DisableDefaultControl();
    }
    template<typename T>
    void DisableFeature(PhoXiFeature<T> &Feature) {
        Feature.Disable();
    }
    template<typename T>
    void SetFeatureValue(PhoXiFeature<T> &Feature, const T &Value) {
        Feature.SetStoredValue(Value);
    }
    template<typename T>
    void SetFeatureErrorMessage(PhoXiFeature<T> &Feature, const std::string &Error) {
        Feature.SetErrorMessage(Error);
    }
    virtual bool StartPhoXiAcquisition();
    virtual bool StopPhoXiAcquisition();
    virtual bool LogOut();
    virtual int TriggerDeviceFrame(bool WaitForAccept = true, bool WaitForGrabbingEnd = false, const std::string& CustomMessage = "");
    virtual bool SaveLastDeviceOutput(const std::string &FilePath, int FrameID = -1);
    bool EvaluateTimeout(PhoXiTimeout &Timeout);
    virtual bool CheckConnectionStatus() = 0;
    const int FEATURE_NOT_IMPLEMENTED = -1001;

#define PHOXI_FEATURE(FEATURE_NAME, FEATURE_TYPE, FEATURE_TYPE_INTERNAL) PHOXI_DECLARE_FEATURE_FUNCTIONS(FEATURE_NAME, FEATURE_TYPE)
  PHOXI_FEATURE_LIST()
#undef PHOXI_FEATURE

    struct SyncLock {
        bool IsActive;
        bool IsStatusAcquired;
        SyncLock();
    }; 
    class SyncGuard {   // guard which ensures HW synchronization is called only once per scope
    private:
        bool clearWhenDestroyed;
        SyncLock& lock;
        SyncGuard(const SyncGuard&) = delete;

    public:
        SyncGuard(SyncLock& syncLock);
        ~SyncGuard();
    };
    SyncLock syncLock;
};

#undef PHOXI_DECLARE_FEATURE
}
}

#endif //_PHOXI_INTERFACE_H