#pragma once
#ifndef _PHOXI_FEATURE_H
#define _PHOXI_FEATURE_H
#include "PhoXiCompilerDefines.h"


#include <functional>
#include <memory>
#include "PhoXiFeatureTypes.h"

namespace pho {
namespace api {
//Forward declaration
template<typename T>
class PhoXiFeature;

///PhoXiFeatureSetGet<T> is a Temporary container for the Value of PhoXiFeature<T>.
/**
PhoXiFeatureSetGet<T> is a Temporary container for the Value of PhoXiFeature<T>.
It allows the Value of PhoXiFeature<T> to be referenced by -> and after the
PhoXiFeatureSetGet<T> goes out of scope, it call SetValue() on PhoXiFeature<T>
if the content of PhoXiFeatureSetGet<T> was modified.
*/
template<typename T>
class PHOXI_DLL_API PhoXiFeatureSetGet : public T {
  private:
    T Original;
    PhoXiFeature<T> *PhoXiFeatureSetGetOwner;
  public:
    PhoXiFeatureSetGet(const T &Other, PhoXiFeature<T> *Owner);
    ~PhoXiFeatureSetGet();
};

//Forward declaration
class PhoXiFeatureInterface;

//Forward declaration
class PhoXiInterface;

class PHOXI_DLL_API PhoXiFeatureInterface {
  public:
    ///Tells if specific feature is implemented for this camera.
    /**
    Some features can be Hardware dependent and could be present only for
    specific families of products. If the feature is not implemented for
    a Device Type, no instance of the family will have the feature.
    @return true if the feature is implemented for the specific device
    type, returns false otherwise.
    */
    bool isImplemented() const;
    ///Tells if specific feature is enabled for this camera
    /**
    Some features can be implemented for a specific Device class, but are
    not enabled because of a minor difference between the instances of the
    same family.
    @return true if the feature is enabled for the specific device, returns
    false otherwise.
    */
    bool isEnabled() const;
    ///Returns last error message from last assign or read of the feature value.
    /**
    call to GetLastErrorMessage can also be used to determine the error state.
    If the returned string is empty, there was no error.
    @return last error message or "" (empty string) if the last operation was
    successful.
    */
    std::string GetLastErrorMessage() const;
    ///Tells if the last assign or read of the feature was successful.
    /**
    @return true if the last operation was successful, returns false otherwise.
    */
    bool isLastOperationSuccessful() const;
    ///Tells if the Local feature value was already initiated.
    /**
    @return true, if this->Value was Initiated, returns false otherwise.
    */
    bool isInitiated() const;
    ///Get the Name of the feature.
    /**
    This name can be used in the FeatureDatabase object.
    @return Feature Name.
    */
    std::string GetName() const;
    ///Tells if the feature is readable.
    virtual bool CanGet() const;
    ///Tells if the feature is settable.
    virtual bool CanSet() const;
  protected:
    bool Implemented;
    bool Enabled;
    std::string Error;
    bool Gettable, Settable;
    std::string Name;
    std::string TypeName;
    PhoXiInterface *Owner;
    bool Initiated;
    /*
    If the DefaultControl is true, the Feature will directly get/set the
    this->Value while Feature is Implemented, Enabled, Gettable/Settable
    and PhoXi is not Connected.
    */
    bool DefaultControl;
    void SetOwner(PhoXiInterface *Owner);
    bool isOwnerConnected() const;
    std::string GetOwnersName() const;
    PhoXiFeatureInterface();
    void Enable();
    void SetAccessibility(bool CanGet, bool CanSet);
    void Enable(bool CanGet, bool CanSet);
    void EnableGet();
    void EnableSet();
    void DisableGet();
    void DisableSet();
    void Disable();
    void EnableDefaultControl();
    void DisableDefaultControl();
    void SetErrorMessage(const std::string &Error);
    void SetName(const std::string &Name);
    void SetTypeName(const std::string &TypeName);
    void ErrorMessage(const std::string &Message) const;
};

template<class T>
class PhoXiCommunicationFeatureCommandsInterface;
template<class T, class TInternal>
class PhoXiCommunicationFeatureCommands;

template<typename T>
class PHOXI_DLL_API PhoXiFeature : public PhoXiFeatureInterface {
  public:
    typedef T(PhoXiInterface::*GetFunction)(PhoXiFeature<T> &);
    typedef bool (PhoXiInterface::*SetFunction)(const T &, PhoXiFeature<T> &);
  public:
    ///Tells if the feature is readable.
    virtual bool CanGet() const override;
    ///Tells if the feature is settable.
    virtual bool CanSet() const override;
    ///Read the value.
    T GetValue();
    ///Set the value.
    bool SetValue(const T &Value);
    ///Get value from the local storage.
    /**
    Some features does always communicates with the device Hardware,
    while others do cache the data locally.
    @return Locally Stored Value.
    */
    T GetStoredValue() const;
    PhoXiFeature<T> &operator=(const PhoXiFeature<T> &Other);
    bool operator=(const T &Value);
    operator T();
    ///Pointer to a internal substructure.
    /**
    Multiple features have more complex structures that just value.
    For Example, if it is desired to change the ShutterMultiplier:
        PhoXiCapturingSettings LocalSettings = PhoXiDevice->CapturingSettings;
        LocalSettings.ShutterMultipler = NewValue;
        PhoXiDevice->CapturingSettings = LocalSettings;
        -While this is inconvenient, you can do
        PhoXiDevice->CapturingSettings->ShutterMultiplier = NewValue; --> this
        will translate to similar behaviour as described before automatically.
    */
    std::unique_ptr <PhoXiFeatureSetGet<T>> operator->();
    bool operator==(const T &Other);
    bool operator!=(const T &Other);
  private:
    T Value;
    std::function<T(PhoXiFeature<T> &)> Get;
    std::function<bool(const T &, PhoXiFeature<T> &)> Set;
    void BindGetFunction(std::function<T(PhoXiFeature<T> &)> Get);
    void BindGetFunction(GetFunction Get, PhoXiInterface *Owner);
    void BindSetFunction(std::function<bool(const T &, PhoXiFeature<T> &)> Set);
    void BindSetFunction(SetFunction Set, PhoXiInterface *Owner);
    void BindFunctions(std::function<T(PhoXiFeature<T> &)> Get, std::function<bool(const T &, PhoXiFeature<T> &)> Set);
    void BindFunctions(GetFunction Get, SetFunction Set, PhoXiInterface *Owner);
    PhoXiFeature(const PhoXiFeature<T> &Other);
    PhoXiFeature();
    void SetStoredValue(const T &Value);
    friend class PhoXiCommunicationFeatureCommandsInterface<T>;
    //template<class TInternal>
    //friend class PhoXiCommunicationFeatureCommands<T, TInternal>;
    friend class PhoXiInterface;
    friend class PhoXiRawAccessHandlerTools;
};
}
}

#endif //_PHOXI_FEATURE_H
