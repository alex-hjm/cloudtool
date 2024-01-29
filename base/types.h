/**
 * @file types.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-28
 */
#ifndef __BASE_TYPES_H__
#define __BASE_TYPES_H__

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <QMetaType>

#include "export.h"

CT_BEGIN_NAMESPACE

typedef pcl::PointXYZRGBNormal PointXYZRGBN;

enum LogLevel { 
    LOG_INFO, 
    LOG_ERROR, 
    LOG_WARN 
};

struct RGB {
    RGB() {}
    RGB(int r_, int g_, int b_) : r(r_), g(g_), b(b_) {}
    double rf() const { return (double)r / 255; }
    double gf() const { return (double)g / 255; }
    double bf() const { return (double)b / 255; }
    int r;
    int g;
    int b;
};

namespace Color {
    const RGB White{255, 255, 255};
    const RGB Black{0, 0, 0};
    const RGB Red{255, 0, 0};
    const RGB Green{0, 255, 0};
    const RGB Blue{0, 0, 255};
    const RGB Yellow{255, 255, 0};
    const RGB Cyan{0, 255, 255};
    const RGB Purple{255, 0, 255};
    const RGB Grey{128, 128, 128};
}  

struct BBox {
    double width;
    double height;
    double depth;
    Eigen::Affine3f pose;
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
};

CT_END_NAMESPACE
#endif  // __BASE_TYPES_H__