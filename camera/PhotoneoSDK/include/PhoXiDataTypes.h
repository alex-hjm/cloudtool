#pragma once
#ifndef _PHOXI_DATA_TYPES_H
#define _PHOXI_DATA_TYPES_H
#include "PhoXiCompilerDefines.h"

#include <string>
#include <vector>
#include <array>
#include <stdint.h>
#include <memory>
#include <functional>
#include <cstring>

namespace pho {
namespace api {

typedef int32_t int32_t;
typedef float float32_t;
typedef double float64_t;

#pragma pack(push, 1)
///Simple type for storing rectangular dimensions such Resolution.
/**
The object is used to represent size of 2D elements. Width and Height
are baseic types like int/double.
*/
template<class T>
class PHOXI_DLL_API PhoXiSize_ {
  public:
    T Width;
    T Height;
    PhoXiSize_();
    PhoXiSize_(T _Width, T _Height);
    PhoXiSize_(const PhoXiSize_<T> &Other);
    PhoXiSize_ &operator=(const PhoXiSize_<T> &Other);
    bool operator==(const PhoXiSize_<T> &Other) const;
    bool operator!=(const PhoXiSize_<T> &Other) const;
    T Area() const;
};
typedef PhoXiSize_<int32_t> PhoXiSize;
typedef PhoXiSize_<float64_t> PhoXiSize_64f;
#pragma pack(pop)

#pragma pack(push, 1)
///Status of the device used before connection.
class PHOXI_DLL_API PhoXiConnectionStatus {
  public:
    ///Tells if the device is attached to PhoXi Control.
    bool Attached;
    ///Tells if the device is ready for connection.
    bool Ready;
    PhoXiConnectionStatus();
    bool operator=(const PhoXiConnectionStatus &Other);
};
#pragma pack(pop)
#pragma pack(push, 1)
///Single element of channel of a Matrix of a specified Type.
/**
The object is used as a wrapper for basic types like uchar/uint/float/double.
It provides additional functionality like TypeName, or TypeSize.
*/
template<class DataType>
class MatType {
  private:
    DataType Data;
  public:
    static const int TypeSize = sizeof(DataType);
    static const std::string PHOXI_DLL_API TypeName;
    operator DataType &() { return Data; }
    operator const DataType &() const { return Data; }
    MatType() : Data() {}
    MatType(const MatType<DataType> &Other) : Data(Other.Data) {}
    MatType(const DataType &Other) : Data(Other) {}
    MatType<DataType> operator=(const MatType<DataType> &Other) {
        Data = Other.Data;
        return *this;
    }
    MatType<DataType> operator=(const DataType &Other) {
        Data = Other;
        return *this;
    }
    bool operator==(const MatType &Other) const {
        return (Data == Other.Data);
    }
    bool operator!=(const MatType &Other) const {
        return (Data != Other.Data);
    }
};
#pragma pack(pop)

//Forward declaration
template<class DataType>
const int MatType<DataType>::TypeSize;

#pragma pack(push, 1)
///Interface of a Matrix element object with specified Channels count.
/**
The matrix element is used to carry multi, or single channel types, like XYZ
for the PointCloud. The interface does not carry any data, just the information
about the type.
*/
template<class DataType, int ChannelCount>
class MatElementInterface {
  public:
    static const int ElementChannelCount = ChannelCount;
    static const int ElementSize = MatType<DataType>::TypeSize * ChannelCount;
    static const int ElementChannelSize = MatType<DataType>::TypeSize;
    typedef DataType ElementChannelType;
    static std::string GetElementName() {
        return MatType<DataType>::TypeName + "C" + std::to_string(ChannelCount);
    }
};
#pragma pack(pop)

template<class DataType, int ChannelCount>
const int MatElementInterface<DataType, ChannelCount>::ElementChannelCount;
template<class DataType, int ChannelCount>
const int MatElementInterface<DataType, ChannelCount>::ElementSize;
template<class DataType, int ChannelCount>
const int MatElementInterface<DataType, ChannelCount>::ElementChannelSize;

#pragma pack(push, 1)
///Three channel Matrix element.
/**
3 Dimensional Point or Vector of a specified ChannelType.
*/
template<class ChannelType>
class Point3 : public MatElementInterface<ChannelType, 3> {
  public:
    typedef ChannelType ElementChannelType;
    MatType<ChannelType> x;
    MatType<ChannelType> y;
    MatType<ChannelType> z;
    Point3() : x(ChannelType()), y(ChannelType()), z(ChannelType()) {}
    Point3(const ChannelType &_x, const ChannelType &_y, const ChannelType &_z) : x(_x), y(_y), z(_z) {}
    bool operator==(const Point3 &Other) const {
        return (x == Other.x && y == Other.y && z == Other.z);
    }
    bool operator!=(const Point3 &Other) const {
        return (x != Other.x || y != Other.y || z != Other.z);
    }
};
typedef Point3<float32_t> Point3_32f;
typedef Point3<float64_t> Point3_64f;
#pragma pack(pop)
#pragma pack(push, 1)
///single channel Matrix element.
template<class ChannelType>
class Scalar : public MatElementInterface<ChannelType, 1>, public MatType<ChannelType> {
  public:
    typedef ChannelType ElementChannelType;
    Scalar() : MatType<ChannelType>() {}
    Scalar(const Scalar &Other) : MatType<ChannelType>(Other) {}
    Scalar(const ChannelType &Other) : MatType<ChannelType>(Other) {}
};

typedef Scalar<uint8_t> Intensity_8;
typedef Scalar<uint16_t> Intensity_16;
typedef Scalar<float32_t> Intensity_32f;
typedef Scalar<float32_t> Depth_32f;
typedef Scalar<float64_t> Depth_64f;
#pragma pack(pop)

#pragma pack(push, 1)
///Simple type for storing rectangular dimensions such Resolution.
template<class ChannelType>
class AxisVolume {
public:
    Point3<ChannelType> min, max;
    AxisVolume() {
        min = Point3<ChannelType>(0.0, 0.0, 0.0);
        max = Point3<ChannelType>(0.0, 0.0, 0.0);
    }
    AxisVolume(const Point3<ChannelType>& _min, const Point3<ChannelType>& _max) {
        min = _min;
        max = _max;
    }
    AxisVolume(const AxisVolume &Other) {
        min = Other.min;
        max = Other.max;
    }
    AxisVolume &operator=(const AxisVolume &Other) {
        min = Other.min;
        max = Other.max;
        return *this;
    }
    bool operator==(const AxisVolume &Other) const {
        return min == Other.min && max == Other.max;
    }
};
typedef AxisVolume<float32_t> AxisVolume_32f;
typedef AxisVolume<float64_t> AxisVolume_64f;
#pragma pack(pop)

#pragma pack(push, 1)
///3 channel matrix element for storing information about bgr color.
template<class ChannelType>
class ColorBGR : public MatElementInterface<ChannelType, 3> {
  public:
    MatType<ChannelType> b;
    MatType<ChannelType> g;
    MatType<ChannelType> r;
    ColorBGR() : b(ChannelType()), g(ChannelType()), r(ChannelType()) {}
    ColorBGR(const ChannelType &_b, const ChannelType &_g, const ChannelType &_r) : b(_b), g(_g), r(_r) {}
    bool operator==(const ColorBGR &Other) const {
        return (b == Other.b && g == Other.g && r == Other.r);
    }
    bool operator!=(const ColorBGR &Other) const {
        return (b != Other.b || g != Other.g || r != Other.r);
    }
};

typedef ColorBGR<uint8_t> ColorBGR_8;
typedef ColorBGR<float32_t> ColorBGR_32f;

#pragma pack(pop)

#pragma pack(push, 1)
///3 channel matrix element for storing information about bgr color.
template<class ChannelType>
class ColorRGB : public MatElementInterface<ChannelType, 3> {
public:
    MatType<ChannelType> r;
    MatType<ChannelType> g;
    MatType<ChannelType> b;
    ColorRGB() : r(ChannelType()), g(ChannelType()), b(ChannelType()) {}
    ColorRGB(const ChannelType &_r, const ChannelType &_g, const ChannelType &_b) : r(_r), g(_g), b(_b) {}
    bool operator==(const ColorRGB &Other) const {
        return (b == Other.b && g == Other.g && r == Other.r);
    }
    bool operator!=(const ColorRGB &Other) const {
        return (b != Other.b || g != Other.g || r != Other.r);
    }
};

typedef ColorRGB<uint8_t> ColorRGB_8;
typedef ColorRGB<float32_t> ColorRGB_32f;

#pragma pack(pop)

template<class T>
struct MatInterfaceNoDelete {
    void operator()(T *ptr) const {
    }
};

template<class T>
void MatInterfaceNoDeleteAction(T Ptr[]) {
    return;
}

#pragma pack(push, 1)
///Interface of a Matrix object.
/**
Matrices are used for storing Frame outputs such as PointClouds
or Textures. General n-dimensional Matrix interface.
*/
template<class ElementType, int Dimension>
class MatInterface {
  public:
    enum OwnershipTransfer {
        CopyData,
        NoCopyNoOwnership,
        NoCopyFullOwnership
    };
  protected:
    std::unique_ptr <ElementType, std::function<void(ElementType *)>> Data;
    std::size_t ElementsCount;
    std::size_t DimensionSize[Dimension];
    std::size_t StepSize[Dimension];
    void SetSizeND(const std::vector<int> &SizeND) {
        StepSize[Dimension - 1] = 1;
        for (std::size_t i = 0; i < SizeND.size(); i++) {
            DimensionSize[i] = SizeND[i];
        }
        for (int i = Dimension - 2; i >= 0; i--) {
            StepSize[i] = StepSize[i + 1] * DimensionSize[i + 1];
        }
    }
    std::size_t GetElementsCountInternal() const {
        std::size_t Result = DimensionSize[0];
        for (std::size_t i = 1; i < Dimension; i++) {
            Result *= DimensionSize[i];
        }
        return Result;
    }
    std::size_t GetElementsCountInternal(const std::vector<int> &SizeND) const {
        std::size_t Result = SizeND[0];
        for (std::size_t i = 1; i < SizeND.size(); i++) {
            Result *= SizeND[i];
        }
        return Result;
    }
    /**
    Reshape the Matrix to be n-dimensional SizeND[0], SizeND[1], ... SizeND[n-1].
    The Data will not be touched - this is O(1) operation.
    */
    bool ReshapeInternal(const std::vector<int> &SizeND) {
        if ((int) SizeND.size() != Dimension) return false;
        std::size_t ElementsCountNew = GetElementsCountInternal(SizeND);
        if (ElementsCountNew != ElementsCount) return false;
        SetSizeND(SizeND);
        return true;
    }
    /**
    Reshape the Matrix to be n-dimensional SizeND[0], SizeND[1], ... SizeND[n-1]
    and reallocate data if needed.
    */
    bool ResizeInternal(const std::vector<int> &SizeND) {
        if ((int) SizeND.size() != Dimension) return false;
        std::size_t ElementsCountNew = GetElementsCountInternal(SizeND);
        Data = std::unique_ptr < ElementType, std::function
            < void(ElementType * ) >> (new ElementType[ElementsCountNew], [](ElementType *Ptr) { delete[] Ptr; });
        ElementsCount = ElementsCountNew;
        //Data.resize(ElementsCount);
        return ReshapeInternal(SizeND);
    }
    bool AssignInternal(const std::vector<int> &SizeND, ElementType *Data_, OwnershipTransfer DataOwnershipTransfer) {
        switch (DataOwnershipTransfer) {
            case OwnershipTransfer::CopyData:
                if (!Resize(SizeND)) return false;
                std::memcpy(GetDataPtr(), Data_, GetDataSize());
                break;
            case OwnershipTransfer::NoCopyNoOwnership:
                SetSizeND(SizeND);
                Data = std::unique_ptr < ElementType, std::function
                    < void(ElementType * ) >> (Data_, [](ElementType *Ptr) {});
                ElementsCount = GetElementsCountInternal();
                break;
            case OwnershipTransfer::NoCopyFullOwnership:
                SetSizeND(SizeND);
                Data = std::unique_ptr < ElementType, std::function
                    < void(ElementType * ) >> (Data_, [](ElementType *Ptr) { delete[] Ptr; });
                ElementsCount = GetElementsCountInternal();
                break;
        }
        return ReshapeInternal(SizeND);
    }
  public:
    ///Get a list of all dimension sizes.
    std::vector<int> GetDimensions() const {
        std::vector<int> Result((std::size_t) Dimension);
        for (int i = 0; i < Dimension; i++) {
            Result[i] = (int) DimensionSize[i];
        }
        return Result;
    }
    ///Get i-th dimension size.
    int GetDimension(int Index) const {
        return (int) DimensionSize[Index];
    }
    ///Get the number of Matrix elements.
    std::size_t GetElementsCount() const {
        return ElementsCount;
    }
    ///Get Data Size of whole Matrix in Bytes.
    std::size_t GetDataSize() const {
        return GetElementsCount() * sizeof(ElementType);
    }
    ///Get the size of the Matrix Element in bytes.
    static std::size_t GetElementSize() {
        return sizeof(ElementType);
    }
    ///Get the Name of the Matrix Element.
    static std::string GetElementName() {
        return ElementType::GetElementName();
    }
    virtual bool Reshape(const std::vector<int> &SizeND) = 0;
    virtual bool Resize(const std::vector<int> &SizeND) = 0;
    virtual bool Assign(const std::vector<int> &SizeND,
                        ElementType *Data_,
                        OwnershipTransfer DataOwnershipTransfer) = 0;
    virtual ElementType &At(const std::vector<int> &Index) {
        std::size_t ArrayIndex = 0;
        for (std::size_t i = 0; i < Index.size(); i++) {
            ArrayIndex += Index[i] * StepSize[i];
        }
        return Data.get()[ArrayIndex];
    }
    virtual const ElementType &At(const std::vector<int> &Index) const {
        std::size_t ArrayIndex = 0;
        for (std::size_t i = 0; i < Index.size(); i++) {
            ArrayIndex += Index[i] * StepSize[i];
        }
        return Data.get()[ArrayIndex];
    }
    MatInterface(const MatInterface<ElementType, Dimension> &Other) : ElementsCount(0) {
        if (ResizeInternal(Other.GetDimensions())) {
            std::memcpy(GetDataPtr(), Other.GetDataPtr(), GetDataSize());
        }
    }
    MatInterface(const std::vector<int> &SizeND) : ElementsCount(0) {
        ResizeInternal(SizeND);
    }
    MatInterface(const std::vector<int> &SizeND, ElementType *Data, OwnershipTransfer DataOwnershipTransfer)
        : ElementsCount(0) {
        AssignInternal(SizeND, Data, DataOwnershipTransfer);
    }
    MatInterface() : ElementsCount(0) {
        for (int i = 0; i < Dimension; i++) {
            DimensionSize[i] = 0;
            StepSize[i] = 0;
        }
        StepSize[Dimension - 1] = 1;
    }
    bool operator=(const MatInterface<ElementType, Dimension> &Other) {
        if (Resize(Other.GetDimensions())) {
            std::memcpy(GetDataPtr(), Other.GetDataPtr(), GetDataSize());
            return true;
        }
        return false;
    }
    bool operator=(const ElementType &Element) {
        std::fill(Data[0], Data[ElementsCount - 1], Element);
        return true;
    }
    ///Get pointer to Raw data.
    ElementType *GetDataPtr() {
        if (!Data) return NULL;
        return Data.get();
    }
    ///Get const pointer to Raw data.
    const ElementType *GetDataPtr() const {
        if (!Data) return NULL;
        return Data.get();
    }
    ///Returns true if the Matrix does not contain data.
    bool Empty() const {
        return (ElementsCount == 0);
    }
    ///Clears the Matrix data and size.
    bool Clear() {
        for (int i = 0; i < Dimension; i++) {
            DimensionSize[i] = 0;
            StepSize[i] = 0;
        }
        StepSize[Dimension - 1] = 1;
        Data = nullptr;
        ElementsCount = 0;
        return true;
    }
    std::size_t GetStepSize(int Dimenstion) const {
        return StepSize[Dimenstion];
    }
};

#pragma pack(pop)

#pragma pack(push, 1)
///2D Matrix.
/**
Matrices are used for storing Frame outputs such as PointClouds or Textures.
Most common matrix type, widely used for organized Frame like structures.
*/
template<class ElementType>
class Mat2D : public MatInterface<ElementType, 2> {
  private:
    void UpdateSizeInternal() {
        Size.Height = (int) MatInterface<ElementType, 2>::DimensionSize[0];
        Size.Width = (int) MatInterface<ElementType, 2>::DimensionSize[1];
    }
    std::vector<int> PhoXiSize2SizeND(const PhoXiSize &Size) const {
        std::vector<int> Result(2);
        Result[0] = Size.Height;
        Result[1] = Size.Width;
        return Result;
    }
  public:
    ///Read-only size information.
    PhoXiSize Size;
    /**
    Reshape the Matrix to be n-dimensional SizeND[0], SizeND[1], ... SizeND[n-1].
    The Data will not be touched - this is O(1) operation.
    */
    virtual bool Reshape(const std::vector<int> &SizeND) override {
        bool Result = MatInterface<ElementType, 2>::ReshapeInternal(SizeND);
        if (Result) {
            UpdateSizeInternal();
        }
        return Result;
    }
    /**
    Reshape the matrix to be 2 dimensional Size.Height x Size.Width.
    The Data will not be touched - this is O(1) operation.
    */
    virtual bool Reshape(const PhoXiSize &Size) {
        bool Result = MatInterface<ElementType, 2>::ReshapeInternal(PhoXiSize2SizeND(Size));
        if (Result) {
            UpdateSizeInternal();
        }
        return Result;
    }
    /**
    Reshape the Matrix to be n-dimensional SizeND[0], SizeND[1], ... SizeND[n-1]
    and reallocate data if needed.
    */
    virtual bool Resize(const std::vector<int> &SizeND) override {
        bool Result = MatInterface<ElementType, 2>::ResizeInternal(SizeND);
        if (Result) {
            UpdateSizeInternal();
        }
        return Result;
    }
    /**
    Reshape the matrix to be 2 dimensional Size.Height x Size.Width
    and reallocate data if needed.
    */
    virtual bool Resize(const PhoXiSize &Size) {
        bool Result = MatInterface<ElementType, 2>::ResizeInternal(PhoXiSize2SizeND(Size));
        if (Result) {
            UpdateSizeInternal();
        }
        return Result;
    }
    ///Access to a specified matrix element of 2D matrix.
    virtual ElementType &At(int y, int x) {
        return MatInterface<ElementType, 2>::Data.get()[y * MatInterface<ElementType, 2>::StepSize[0] + x];
    }
    ///Access to a specified matrix element of 2D matrix by const reference.
    virtual const ElementType &At(int y, int x) const {
        return MatInterface<ElementType, 2>::Data.get()[y * MatInterface<ElementType, 2>::StepSize[0] + x];
    }
    ///Access to a specified matrix row of 2D matrix.
    ElementType *operator[](int y) {
        return &MatInterface<ElementType, 2>::Data.get()[y * MatInterface<ElementType, 2>::StepSize[0]];
    }
    ///Access to a specified matrix row of 2D matrix by const reference.
    const ElementType *operator[](int y) const {
        return &MatInterface<ElementType, 2>::Data.get()[y * MatInterface<ElementType, 2>::StepSize[0]];
    }
    ///Get the data copy of specified channel alone.
    bool GetChannelCopy(Mat2D<Scalar<typename ElementType::ElementChannelType>> &OutputMat, int ChannelIndex) const {
        if (ChannelIndex >= ElementType::ElementChannelCount) {
            return false;
        }
        if (!OutputMat.Resize(Size)) return false;
        Scalar<typename ElementType::ElementChannelType> *OutputPixel;
        const Scalar<typename ElementType::ElementChannelType> *InputPixel;
        for (decltype(Size.Height) y = 0; y < Size.Height; y++) {
            InputPixel = (const Scalar<typename ElementType::ElementChannelType> *) &At(y, 0);
            InputPixel += ChannelIndex;
            OutputPixel = &OutputMat.At(y, 0);
            for (decltype(Size.Width) x = 0; x < Size.Width; x++) {
                *OutputPixel = *InputPixel;
                ++OutputPixel;
                InputPixel += ElementType::ElementChannelCount;
            }
        }
        return true;
    }
    Mat2D(const Mat2D &Other) : MatInterface<ElementType, 2>(Other) { UpdateSizeInternal(); }
    Mat2D(const std::vector<int> &SizeND) : MatInterface<ElementType, 2>(SizeND) { UpdateSizeInternal(); }
    Mat2D(const std::vector<int> &SizeND,
          ElementType *_Data,
          typename MatInterface<ElementType, 2>::OwnershipTransfer DataOwnershipTransfer) : MatInterface<ElementType,
                                                                                                         2>(SizeND,
                                                                                                            _Data,
                                                                                                            DataOwnershipTransfer) { UpdateSizeInternal(); }
    Mat2D(const PhoXiSize &Size) : MatInterface<ElementType, 2>(PhoXiSize2SizeND(Size)) { UpdateSizeInternal(); }
    Mat2D(const PhoXiSize &Size,
          ElementType *_Data,
          typename MatInterface<ElementType, 2>::OwnershipTransfer DataOwnershipTransfer) : MatInterface<ElementType,
                                                                                                         2>(
        PhoXiSize2SizeND(Size),
        _Data,
        DataOwnershipTransfer) { UpdateSizeInternal(); }
    Mat2D() : MatInterface<ElementType, 2>() { UpdateSizeInternal(); }

    ///Assign data from an external source.
    /**
    You can choose of multiple types of DataOwnershipTransfer.
    CopyData - Data will be copied and freed in the object destructor.
    NoCopyNoOwnership - Data will reference to remote pointer and the data will
        not be freed in the object destructor.
    NoCopyFullOwnership - Data will reference to remote pointer and the data
        will be copied in the object destructor.
    */
    virtual bool Assign(const PhoXiSize &Size,
                        ElementType *_Data,
                        typename MatInterface<ElementType, 2>::OwnershipTransfer DataOwnershipTransfer) {
        const bool
            Result = MatInterface<ElementType, 2>::AssignInternal(PhoXiSize2SizeND(Size), _Data, DataOwnershipTransfer);
        UpdateSizeInternal();
        return Result;
    }
    ///Assign data from an external source - general case.
    /**
    You can choose of multiple types of DataOwnershipTransfer.
    CopyData - Data will be copied and freed in the object destructor.
    NoCopyNoOwnership - Data will reference to remote pointer and the data
        will not be freed in the object destructor.
    NoCopyFullOwnership - Data will reference to remote pointer and the data
    will be copied in the object destructor.
    */
    virtual bool Assign(const std::vector<int> &SizeND,
                        ElementType *_Data,
                        typename MatInterface<ElementType, 2>::OwnershipTransfer DataOwnershipTransfer) {
        return MatInterface<ElementType, 2>::AssignInternal(SizeND, _Data, DataOwnershipTransfer);
    }

#ifdef PHOXI_OPENCV_SUPPORT
    /**
    * Convert `Mat2D` to OpenCV `cv::Mat` object - using copy.
    *
    * @param Destination [in,out] Output parameter, will be reallocated
    * @return `true` on Success, `false` otherwise.
    */
    bool ConvertTo(cv::Mat& Destination) const;

    /**
    * Convert OpenCV `cv::Mat` object to `Mat2D` - using copy.
    *
    * @param Source [in] This object will be reallocated if necessary, if the
    * `Source` have incorrect number of channels, the call will fail and return
    * false, otherwise it will be converted, if needed.
    * @return `true` on Success, `false` otherwise.
    */
    bool ConvertFrom(const cv::Mat& Source);
#else
#   define NO_DIRECT_OPENCV_SUPPORT
#endif

    void ConvertTo2DArray(float *mat2DArray, int dim1, int dim2);
    void ConvertTo3DArray(float *mat2DArray, int dim1, int dim2, int dim3);

};
#pragma pack(pop)

template<class ElementType>
void Mat2D<ElementType>::ConvertTo3DArray(float *mat2DArray, int dim1, int dim2, int dim3) {
    for (int y = 0; y < dim1; y++) {
        memcpy(mat2DArray,
               this->operator[](y),
               MatInterface<ElementType, 2>::StepSize[0] * ElementType::ElementSize);
        mat2DArray += dim2*dim3;
    }
}

template<class ElementType>
void Mat2D<ElementType>::ConvertTo2DArray(float *mat2DArray, int dim1, int dim2) {
    for (int y = 0; y < dim1; y++) {
        memcpy(mat2DArray,
               this->operator[](y),
               MatInterface<ElementType, 2>::StepSize[0] * ElementType::ElementSize);
        mat2DArray += dim2;
    }
}

typedef Mat2D<Point3_32f> PointCloud32f;
typedef Mat2D<Point3_32f> NormalMap32f;
typedef Mat2D<Depth_32f> DepthMap32f;
typedef Mat2D<Intensity_32f> ConfidenceMap32f;
typedef Mat2D<Intensity_32f> Texture32f;
typedef Mat2D<ColorRGB_32f> TextureRGB32f;
typedef Mat2D<Depth_64f> RotationMatrix64f;
typedef Mat2D<Depth_64f> CameraMatrix64f;

typedef std::shared_ptr <PointCloud32f> PPointCloud32f;
typedef std::shared_ptr <PointCloud32f> PNormalMap32f;
typedef std::shared_ptr <DepthMap32f> PDepthMap32f;
typedef std::shared_ptr <ConfidenceMap32f> PConfidenceMap32f;
typedef std::shared_ptr <Texture32f> PTexture32f;
typedef std::shared_ptr <TextureRGB32f> PTextureRGB32f;

//Forward Declaration for internal purposes
class PhoXiCoordinateTransformationInternal;
#pragma pack(push, 8)
///Defines transformation of the coordinate space.
class PHOXI_DLL_API PhoXiCoordinateTransformation {
private:
    friend class PhoXiCoordinateTransformationInternal;
    ///Defines if the coordinate transformation is supported.
    bool Supported;
public:
    ///3 x 3 Rotation Matrix.
    RotationMatrix64f Rotation;
    ///3D Translation vector.
    Point3_64f Translation;
    PhoXiCoordinateTransformation();
    bool isSupported() const;
    bool operator=(const PhoXiCoordinateTransformation &Other);
    bool operator==(const PhoXiCoordinateTransformation &Other) const;
};
#pragma pack(pop)

enum TemperatureDeviceSensor {
    ProjectionUnitBoard = 0,
    ProjectionUnitLaser,
    ProjectionUnitGalvo,
    CameraSensorBoard,
    CameraRearHousing,
    CameraFrontHousing,
    CameraSensorDie,
    CameraSensorDieRaw,
    CameraInterfaceBoard,
    TempsCount
};

//Temperature not available.
static const double PHOXI_DEVICE_TEMPERATURE_INVALID = -273.15;

#pragma pack(push, 8)
///Additional frame informations.
class PHOXI_DLL_API FrameInfo {
  public:
    ///Frame index counted from the connection start.
    uint64_t FrameIndex;
    ///Frame start time in ms counted from the connection start.
    double FrameTimestamp;
    ///Frame capturing duration in ms.
    double FrameDuration;
    ///Frame computation duration in ms.
    double FrameComputationDuration;
    ///Frame transfer duration in ms.
    double FrameTransferDuration;
    ///Sensor position in the 3D space.
    Point3_64f SensorPosition;
    ///Sensor 3D Axis in the space.
    Point3_64f SensorXAxis, SensorYAxis, SensorZAxis;
    ///Total device scan count.
    int64_t TotalScanCount;
    ///Array of temperatures from various sensors.
    std::array<double, TemperatureDeviceSensor::TempsCount> Temperatures;
    FrameInfo();
};
#pragma pack(pop)

#pragma pack(push, 8)
///Frame output structure.
class Frame {
  public:
    bool Successful;
    ///Additional informations.
    FrameInfo Info;
    /**
    2 dimensional organized 3 channel PointCloud structure, organized as
    [X mm, Y mm, Z mm] of 32 bit floats, unmeasured point has [0.0, 0.0, 0.0].
    */
    PointCloud32f PointCloud;
    /**
    2 dimensional organized 3 channel NormalMap structure, organized as normalized
    [X, Y, Z] of 32 bit floats, unmeasured normal has [0.0, 0.0, 0.0].
    */
    NormalMap32f NormalMap;
    /**
    2 dimensional 1 channel Depth map of 32 bit floats coding orthogonal distances
    from the internal camera in mm.
    */
    DepthMap32f DepthMap;
    /**
    2 dimensional 1 channel Confidence map of 32 bit floats coding measurement
    confidence.
    */
    ConfidenceMap32f ConfidenceMap;
    ///2 dimensional 1 channel Texture of 32 bit floats coding intensity.
    Texture32f Texture;
    ///2 dimensional 3 channel Texture of 32 bit floats coding RGB values.
    TextureRGB32f TextureRGB;
    ///Contains content of a CustomMessage assigned on TriggerFrame.
    std::string CustomMessage;
    ///Returns true if there is no output, returns false otherwise.
    bool PHOXI_DLL_API Empty() const;
    /**
    Returns resolution of the captured frame - All outputs are either empty,
    or have this resolution.
    */
    PhoXiSize PHOXI_DLL_API GetResolution() const;
    /**
    Creates a shared_ptr of Frame with new allocated data in the local
    address space.
    */
    std::shared_ptr<Frame> Clone() {
        std::shared_ptr<Frame> Result = std::make_shared<Frame>();
        Result->Successful = Successful;
        Result->Info = Info;
        Result->PointCloud = PointCloud;
        Result->NormalMap = NormalMap;
        Result->DepthMap = DepthMap;
        Result->ConfidenceMap = ConfidenceMap;
        Result->Texture = Texture;
        Result->TextureRGB = TextureRGB;
        Result->CustomMessage = CustomMessage;
        return Result;
    }
    ///Save the Frame as an Ply with all satellite data
    /**
    Saves Frame structure as an organized Stanford's PLY with specified Texture
    and Normals. All informations are stored, this can be used as an container.
    @param FilePath Input ply FilePath.
    @param BinaryFile If true, the ply will be stored in binary form, more
    memory efficient (true is default).
    @param NormalizeTexture If true, the RGB information in the PLY will be
    normalized, 32 bit Intensity information will be untouched.
    @param StorePointCloud If true and PointCloud is not empty, the PointCloud
    will be stored into PLY.
    @param StoreNormalMap If true and NormalMap is not empty, the NormalMap
    will be stored into PLY.
    @param StoreDepthMap If true and DepthMap is not empty, the DepthMap will
    be stored into PLY.
    @param StoreTexture If true and Texture is not empty, the Texture will be
    stored into PLY.
    @param StoreConfidenceMap If true and ConfidenceMap is not empty, the
    ConfidenceMap will be stored into PLY.
    @param Unordered If true the Frame will be stored without non-measured
    points.
    @param Metadata If true, every available information will be stored into
    PLY. If false, only vertex elements will be present in saved PLY file
    (HALCON compatible).
    @return true on success.
    */
    bool PHOXI_DLL_API SaveAsPly(
        const std::string &FilePath,
        bool BinaryFile = true,
        bool NormalizeTexture = true,
        bool StorePointCloud = true,
        bool StoreNormalMap = true,
        bool StoreDepthMap = true,
        bool StoreTexture = true,
        bool StoreConfidenceMap = true,
        bool Unordered = false,
        bool Metadata = true) const;

    /**
    Saves Frame structure as an organized Stanford's PLY with specified
    Texture and Normals. All informations are stored, this can be used
    as an container.
    @param FilePath Input ply FilePath.
    @param BinaryFile If true, the ply will be stored in binary form, more
    memory efficient (true is default).
    @param NormalizeTexture If true, the RGB information in the PLY will be
    normalized, 32 bit Intensity information will be untouched.
    @param StorePointCloud If true and PointCloud is not empty, the PointCloud
    will be stored into PLY.
    @param StoreNormalMap If true and NormalMap is not empty, the NormalMap
    will be stored into PLY.
    @param StoreDepthMap If true and DepthMap is not empty, the DepthMap will
    be stored into PLY.
    @param StoreTexture If true and Texture is not empty, the Texture will be
    stored into PLY.
    @param StoreConfidenceMap If true and ConfidenceMap is not empty, the
    ConfidenceMap will be stored into PLY.
    @param Unordered If true the Frame will be stored without non-measured
    points.
    @param cameraCalibrationMatrix std::vector<float>(9).
    @param distortionCoefficientsMatrix std::vector<float>(14).
    @param cameraResolution std::vector<float>(2).
    @param cameraBinning std::vector<float>(2).
    @param Metadata If true, every available information will be stored into
    PLY. If false, only vertex elements will be present in saved PLY file
    (HALCON compatible).
    @return true on success.
    */
    bool PHOXI_DLL_API SaveAsPly(
        const std::wstring &FilePath,
        bool BinaryFile = true,
        bool NormalizeTexture = true,
        bool StorePointCloud = true,
        bool StoreNormalMap = true,
        bool StoreDepthMap = true,
        bool StoreTexture = true,
        bool StoreConfidenceMap = true,
        bool Unordered = false,
        const std::vector<float> &CameraCalibrationMatrix = std::vector<float>(),
        const std::vector<float> &DistortionCoefficientsMatrix = std::vector<float>(),
        const std::vector<float> &CameraResolution = std::vector<float>(),
        const std::vector<float> &CameraBinning = std::vector<float>(),
        bool Metadata = true) const;

    Frame() : Successful(false) {}

#ifdef PHOXI_PCL_SUPPORT
    ///Converts to PCL's PointCloud structure
    /**
    Based on the chosen type, the data will be present in the output
    The data are copied
    */
    template <class PCLPointType>
    bool ConvertTo(pcl::PointCloud<PCLPointType>& Destination, bool NormalizeTexture = true) const;
#  ifdef PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
    ///Converts to PCL's obsolete PointCloud2 structure
    bool ConvertTo(pcl::PCLPointCloud2& Destination) const;
#  endif
#endif
private:
    bool canSavePLY(bool savePointCloud,
                    bool saveNormalMap,
                    bool saveDepthMap,
                    bool saveTexture,
                    bool saveConfidenceMap) const;
};
#pragma pack(pop)
typedef std::shared_ptr <Frame> PFrame;

} // api
} // pho

#endif //_PHOXI_DATA_TYPES_H