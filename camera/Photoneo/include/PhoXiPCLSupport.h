#pragma once
#ifndef _PHOXI_PCL_SUPPORT_H
#define _PHOXI_PCL_SUPPORT_H
#include "PhoXiCompilerDefines.h"

#ifndef PHOXI_PCL_SUPPORT
#    define PHOXI_PCL_SUPPORT
#endif

#include "PhoXiDataTypes.h"

#include <boost/preprocessor/seq/for_each.hpp>
#include <algorithm>

// Define all PCL point types with Intensity
#define PCL_I_POINT_TYPES       \
  (pcl::PointXYZI)              \
  (pcl::PointXYZINormal)

#define CUSTOM_MIN(a, b) (((a) < (b)) ? (a) : (b))

namespace pho {
namespace api {
namespace pcls {

template<class PCLPointType>
inline bool ResizePointCloud(
    const Frame &Source,
    pcl::PointCloud <PCLPointType> &Destination) {
    PhoXiSize Resolution = Source.GetResolution();
    if (Resolution == PhoXiSize(0, 0)) {
        return false;
    }
    //Destination.resize(Resolution.Area());
    Destination = pcl::PointCloud<PCLPointType>(Resolution.Width, Resolution.Height);
    return true;
}

template<class PCLPointType>
inline bool FillPointCloudXYZ(
    const Frame &Source,
    pcl::PointCloud <PCLPointType> &Destination) {
    return false;
}
#define FillPointCloudXYZ_MACRO(FOO0, FOO1, elem) \
            template<>\
            inline bool FillPointCloudXYZ(const Frame& Source, pcl::PointCloud<elem>& Destination) {\
                if (Source.PointCloud.Empty()) return false;\
                for (size_t i = 0; i < Destination.points.size(); ++i) {\
                    memcpy(Destination.points[i].data, &Source.PointCloud[0][i], sizeof(Point3_32f));\
                }\
                return true;\
            }

BOOST_PP_SEQ_FOR_EACH(FillPointCloudXYZ_MACRO, FOO, PCL_XYZ_POINT_TYPES
)

template<class PCLPointType>
inline bool FillPointCloudNormalXYZ(
    const Frame &Source,
    pcl::PointCloud <PCLPointType> &Destination) {
    return false;
}
#define FillPointCloudNormalXYZ_MACRO(FOO0, FOO1, elem) \
            template<>\
            inline bool FillPointCloudNormalXYZ(const Frame& Source, pcl::PointCloud<elem>& Destination) {\
                if (Source.NormalMap.Empty()) return false;\
                for (size_t i = 0; i < Destination.points.size(); ++i) {\
                    memcpy(Destination.points[i].data_n, &Source.NormalMap[0][i], sizeof(Point3_32f));\
                }\
                return true;\
            }

BOOST_PP_SEQ_FOR_EACH(FillPointCloudNormalXYZ_MACRO, FOO, PCL_NORMAL_POINT_TYPES
)

template<class PCLPointType>
inline bool FillPointCloudI(
    const Frame &Source,
    pcl::PointCloud <PCLPointType> &Destination) {
    return false;
}
#define FillPointCloudI_MACRO(r, FOO, elem) \
            template<>\
            inline bool FillPointCloudI(const Frame& Source, pcl::PointCloud<elem>& Destination) {\
                if (Source.Texture.Empty()) return false;\
                for (size_t i = 0; i < Destination.points.size(); ++i) {\
                    Destination.points[i].intensity = Source.Texture[0][i];\
                }\
                return true;\
            }

BOOST_PP_SEQ_FOR_EACH(FillPointCloudI_MACRO, FOO, PCL_I_POINT_TYPES
)

template<class PCLPointType>
inline bool FillPointCloudRGB(
    const Frame &Source,
    pcl::PointCloud<PCLPointType> &Destination,
    bool NormalizeTexture) {
    return false;
}
#define FillPointCloudRGB_MACRO(FOO0, FOO1, elem) \
            template<>\
            inline bool FillPointCloudRGB(const Frame& Source, pcl::PointCloud<elem>& Destination, bool NormalizeTexture) {\
                if (Source.Texture.Empty()) return false;\
                double TextureMultiplier = 1.0;\
                if (NormalizeTexture && Source.Texture.Size.Area() > 0) {\
                    const int SamplesCount = 10000;\
                    int SamplesX = std::sqrt((double)SamplesCount * ((double)Source.Texture.Size.Width / (double)Source.Texture.Size.Height));\
                    int SamplesY = std::sqrt((double)SamplesCount / ((double)Source.Texture.Size.Width / (double)Source.Texture.Size.Height));\
                    std::vector<float> Samples;\
                    int StepY = std::max<int>(1, Source.Texture.Size.Height / SamplesY);\
                    int StepX = std::max<int>(1, Source.Texture.Size.Width / SamplesX);\
                    for (int y = 0; y < Source.Texture.Size.Height; y += StepY) {\
                        for (int x = 0; x < Source.Texture.Size.Height; x += StepY) {\
                            Samples.push_back(Source.Texture[y][x]);\
                        }\
                    }\
                    if (Samples.size() > 10) {\
                        std::sort(Samples.begin(), Samples.end());\
                        TextureMultiplier = (255.0 / Samples[Samples.size() - 1 - Samples.size() / 98]) * 0.9;\
                    }\
                }\
                for (size_t i = 0; i < Destination.points.size(); ++i) {\
                    unsigned char Grey = (unsigned char)CUSTOM_MIN(((float)Source.Texture[0][i] * (float)TextureMultiplier), 255.0f);\
                    Destination.points[i].r = Grey;\
                    Destination.points[i].g = Grey;\
                    Destination.points[i].b = Grey;\
                }\
                return true;\
            }

BOOST_PP_SEQ_FOR_EACH(FillPointCloudRGB_MACRO, FOO, PCL_RGB_POINT_TYPES
)

template<class PCLPointType>
inline bool toPCLPointCloud(
    const Frame &Source,
    pcl::PointCloud<PCLPointType> &Destination,
    bool NormalizeTexture = true) {
    if (!ResizePointCloud(Source, Destination)) {
        return false;
    }
    FillPointCloudXYZ(Source, Destination);
    FillPointCloudNormalXYZ(Source, Destination);
    FillPointCloudI(Source, Destination);
    FillPointCloudRGB(Source, Destination, NormalizeTexture);
    return true;
}

#define PHOXI_FRAME_PCL_SUPPORT \
            template <class PCLPointType>\
            inline bool ConvertTo(pcl::PointCloud<PCLPointType>& Destination, bool NormalizeTexture = true) {\
                return pcls::toPCLPointCloud(*this, Destination);\
            }

} // namespace pcls

template<class PCLPointType>
inline bool Frame::ConvertTo(
    pcl::PointCloud<PCLPointType> &Destination,
    bool NormalizeTexture) const {
    return pcls::toPCLPointCloud(*this, Destination);
}
#  ifdef PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
inline bool Frame::ConvertTo(pcl::PCLPointCloud2& Destination) const {
    if (!NormalMap.Empty()){
        pcl::PointCloud<pcl::PointXYZINormal> Temporary;
        bool Result = ConvertTo(Temporary, false);
        pcl::toPCLPointCloud2(Temporary, Destination);
        return Result;
    } else {
        pcl::PointCloud<pcl::PointXYZI> Temporary;
        bool Result = ConvertTo(Temporary, false);
        pcl::toPCLPointCloud2(Temporary, Destination);
        return Result;
    }
}
#endif

} // namespace api
} // namespace pho

#endif //_PHOXI_PCL_SUPPORT_H