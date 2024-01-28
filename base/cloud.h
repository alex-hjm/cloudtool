/**
 * @file cloud.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-28
 */
#ifndef __BASE_CLOUD_H__
#define __BASE_CLOUD_H__

#include <pcl/point_cloud.h>

#include <QFileInfo>

#include "types.h"

#define CLOUD_BBOX_FLAG       "-bbox"
#define CLOUD_NORMALS_FLAG    "-normals"

CT_BEGIN_NAMESPACE

class CT_EXPORT Cloud : public pcl::PointCloud<PointXYZRGBN> 
{
public:
  Cloud() {}

  Cloud& operator+=(const Cloud& rhs) { concatenate((*this), rhs); return (*this); }
  Cloud operator+(const Cloud& rhs) { return (Cloud(*this) += rhs); }

  using Ptr = std::shared_ptr<Cloud>;
  using ConstPtr = std::shared_ptr<const Cloud>;
  Ptr makeShared() const { return Ptr(new Cloud(*this)); }

  Eigen::Vector3f center() const { return bbox.translation; }
  float volume() const { return bbox.depth * bbox.height * bbox.width; }
  int pointsNum() const { return this->size(); }

public:
  std::string id{"undefined"};
  std::string bbox_id{id + CLOUD_BBOX_FLAG};
  std::string normals_id{id + CLOUD_NORMALS_FLAG};
  BBox bbox;
  RGB bbox_rgb{Color::White};
  RGB normals_rgb{Color::Green};
  int point_size{1};
  float opacity{1.0};
  QFileInfo file_info;
};

CT_END_NAMESPACE
#endif  // __BASE_CLOUD_H__