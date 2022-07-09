#pragma once
#ifndef _PHOXI_OPENCV_SUPPORT_H
#define _PHOXI_OPENCV_SUPPORT_H
#include "PhoXiCompilerDefines.h"

#ifndef PHOXI_OPENCV_SUPPORT
#    define PHOXI_OPENCV_SUPPORT
#endif

#include "PhoXiDataTypes.h"

namespace pho {
namespace api {

template<class ElementType>
inline bool ConvertMat2DToOpenCVMat(
    const pho::api::Mat2D<ElementType>& Source,
    cv::Mat &Destination) {

    switch (ElementType::ElementChannelCount) {
    case 1:
        if (Destination.size() != cv::Size(Source.Size.Width, Source.Size.Height) || Destination.type() != cv::DataType<typename ElementType::ElementChannelType>::type)
            Destination = cv::Mat(cv::Size(Source.Size.Width, Source.Size.Height),
            cv::DataType<typename ElementType::ElementChannelType>::type);
        break;
    case 2:
        if (Destination.size() != cv::Size(Source.Size.Width, Source.Size.Height) || Destination.type() != cv::DataType<cv::Point_<typename ElementType::ElementChannelType>>::type)
            Destination = cv::Mat(cv::Size(Source.Size.Width, Source.Size.Height),
            cv::DataType<cv::Point_<typename ElementType::ElementChannelType>>::type);
        break;
    case 3:
        if (Destination.size() != cv::Size(Source.Size.Width, Source.Size.Height) || Destination.type() != cv::DataType<cv::Point3_<typename ElementType::ElementChannelType>>::type)
            Destination = cv::Mat(cv::Size(Source.Size.Width, Source.Size.Height),
            cv::DataType<cv::Point3_<typename ElementType::ElementChannelType>>::type);
        break;
    default:
        return false;
    }
    std::size_t StepSize = Source.GetStepSize(0);
    for (int y = 0; y < Destination.rows; y++) {
        memcpy(Destination.ptr<typename ElementType::ElementChannelType>(y),
            Source[y],
            StepSize * ElementType::ElementSize);
    }
    return true;
}

template <class ElementType>
inline bool ConvertOpenCVMatToMat2D(const cv::Mat& Source, pho::api::Mat2D<ElementType>& Destination) {
    if (Source.channels() != ElementType::ElementChannelCount) return false;
    if (ElementType::ElementChannelCount < 1 || ElementType::ElementChannelCount > 3) return false;
    cv::Mat SourceConverted = Source;

    if (Source.size() != cv::Size(Destination.Size.Width, Destination.Size.Height)) {
        Destination.Resize(PhoXiSize(SourceConverted.cols, SourceConverted.rows));
    }

    switch (ElementType::ElementChannelCount) {
    case 1:
        if (Source.type() != cv::DataType<typename ElementType::ElementChannelType>::type) {
            SourceConverted = cv::Mat();
            Source.convertTo(SourceConverted, cv::DataType<typename ElementType::ElementChannelType>::type);
        }
        break;
    case 2:
        if (Source.type() != cv::DataType<cv::Point_<typename ElementType::ElementChannelType>>::type) {
            SourceConverted = cv::Mat();
            Source.convertTo(SourceConverted, cv::DataType<cv::Point_<typename ElementType::ElementChannelType>>::type);
        }
        break;
    case 3:
        if (Source.type() != cv::DataType<cv::Point3_<typename ElementType::ElementChannelType>>::type) {
            SourceConverted = cv::Mat();
            Source.convertTo(SourceConverted, cv::DataType<cv::Point3_<typename ElementType::ElementChannelType>>::type);
        }
        break;
    default:
        return false;
    }
    std::size_t StepSize = Destination.GetStepSize(0);
    for (int y = 0; y < SourceConverted.rows; y++) {
        memcpy(Destination[y],
            SourceConverted.ptr<const typename ElementType::ElementChannelType>(y),
            StepSize * ElementType::ElementSize);
    }
    return true;
}

#ifndef NO_DIRECT_OPENCV_SUPPORT
template<class ElementType>
bool pho::api::Mat2D<ElementType>::ConvertTo(cv::Mat &Destination) const {
    switch (ElementType::ElementChannelCount) {
    case 1:
        if (Destination.size() != cv::Size(Size.Width, Size.Height) || Destination.type() != cv::DataType<typename ElementType::ElementChannelType>::type)
            Destination = cv::Mat(cv::Size(Size.Width, Size.Height),
            cv::DataType<typename ElementType::ElementChannelType>::type);
        break;
    case 2:
        if (Destination.size() != cv::Size(Size.Width, Size.Height) || Destination.type() != cv::DataType<cv::Point_<typename ElementType::ElementChannelType>>::type)
            Destination = cv::Mat(cv::Size(Size.Width, Size.Height),
            cv::DataType<cv::Point_<typename ElementType::ElementChannelType>>::type);
        break;
    case 3:
        if (Destination.size() != cv::Size(Size.Width, Size.Height) || Destination.type() != cv::DataType<cv::Point3_<typename ElementType::ElementChannelType>>::type)
            Destination = cv::Mat(cv::Size(Size.Width, Size.Height),
            cv::DataType<cv::Point3_<typename ElementType::ElementChannelType>>::type);
        break;
    default:
        return false;
    }
    for (int y = 0; y < Destination.rows; y++) {
        memcpy(Destination.ptr<typename ElementType::ElementChannelType>(y),
            this->operator[](y),
            MatInterface<ElementType, 2>::StepSize[0] * ElementType::ElementSize);
    }
    return true;
}

template<class ElementType>
bool pho::api::Mat2D<ElementType>::ConvertFrom(const cv::Mat& Source){
    if (Source.channels() != ElementType::ElementChannelCount) return false;
    if (ElementType::ElementChannelCount < 1 || ElementType::ElementChannelCount > 3) return false;
    cv::Mat SourceConverted = Source;

    if (Source.size() != cv::Size(Size.Width, Size.Height)) {
        Resize(PhoXiSize(SourceConverted.cols, SourceConverted.rows));
    }

    switch (ElementType::ElementChannelCount) {
    case 1:
        if (Source.type() != cv::DataType<typename ElementType::ElementChannelType>::type) {
            SourceConverted = cv::Mat();
            Source.convertTo(SourceConverted, cv::DataType<typename ElementType::ElementChannelType>::type);
        }
        break;
    case 2:
        if (Source.type() != cv::DataType<cv::Point_<typename ElementType::ElementChannelType>>::type) {
            SourceConverted = cv::Mat();
            Source.convertTo(SourceConverted, cv::DataType<cv::Point_<typename ElementType::ElementChannelType>>::type);
        }
        break;
    case 3:
        if (Source.type() != cv::DataType<cv::Point3_<typename ElementType::ElementChannelType>>::type) {
            SourceConverted = cv::Mat();
            Source.convertTo(SourceConverted, cv::DataType<cv::Point3_<typename ElementType::ElementChannelType>>::type);
        }
        break;
    default:
        return false;
    }
    for (int y = 0; y < SourceConverted.rows; y++) {
        memcpy(this->operator[](y),
            SourceConverted.ptr<const typename ElementType::ElementChannelType>(y),
            MatInterface<ElementType, 2>::StepSize[0] * ElementType::ElementSize);
    }
    return true;
}
#endif

} // namespace api
} // namespace pho

#endif //_PHOXI_OPENCV_SUPPORT_H