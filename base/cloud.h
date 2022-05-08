/**
 * @file cloud.h
 * @author hjm (hjmalex@163.com)
 * @version 1.0
 * @date 2022-05-08
 *
 * @copyright Copyright (c) 2022 hjm
 *
 */
#ifndef CT_BASE_CLOUD_H
#define CT_BASE_CLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QFileInfo>
#include <QString>

#include "base/exports.h"

namespace ct {

struct Box {
  double width = 0;
  double height = 0;
  double depth = 0;
  Eigen::Affine3f pose = Eigen::Affine3f::Identity();
  Eigen::Vector3f translation = Eigen::Vector3f::Zero();
  Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
};

typedef pcl::PointXYZRGBNormal PointXYZRGBN;

class CT_EXPORT Cloud : public pcl::PointCloud<PointXYZRGBN> {
 public:
  Cloud()
      : point_size_(1),
        opacity_(1.0),
        resolution_(0.0f),
        has_normals_(false),
        box_rgb_(255, 255, 255),
        normals_rgb_(255, 255, 255) {}

  using Ptr = std::shared_ptr<Cloud>;
  using ConstPtr = std::shared_ptr<const Cloud>;

  Ptr makeShared() const { return Ptr(new Cloud(*this)); }

  /**
   * @brief 点云包围盒
   */
  Box box() const { return box_; }

  /**
   * @brief 点云ID
   */
  QString id() const { return id_; }

  /**
   * @brief 点云法线ID
   */
  QString normalId() const { return id_ + "_normals"; }

  /**
   * @brief 点云包围盒ID
   */
  QString boxId() const { return id_ + "_box"; }

  /**
   * @brief 点云包围盒颜色
   */
  pcl::RGB boxColor() const { return box_rgb_; }

  /**
   * @brief 点云法线颜色
   */
  pcl::RGB normalColor() const { return normals_rgb_; }

  /**
   * @brief 点云中心
   */
  Eigen::Vector3f center() const { return box_.translation; }

  /**
   * @brief 点云类型
   */
  QString type() const { return type_; }

  /**
   * @brief 点云文件信息
   */
  QFileInfo fileInfo() const { return file_info_; }

  /**
   * @brief 点云文件格式
   */
  QString fileType() const { return file_info_.suffix(); }

  /**
   * @brief 点云文件路径
   */
  QString filePath() const { return file_info_.filePath(); }

  /**
   * @brief 点云文件大小（KB)
   */
  int fileSize() const { return file_info_.size() / 1024; }

  /**
   * @brief 点云点大小
   */
  int pointSize() const { return point_size_; }

  /**
   * @brief 点云透明度
   */
  float opacity() const { return opacity_; }

  /**
   * @brief 点云分辨率
   */
  float resolution() const { return resolution_; }

  /**
   * @brief 点云体积
   */
  float volume() const { return box_.depth * box_.height * box_.width; }

  /**
   * @brief 点云是否有法线
   */
  bool hasNormals() const { return has_normals_; }

  /**
   * @brief 设置点云ID
   */
  void setId(const QString& id) { id_ = id; }

  /**
   * @brief 设置点云包围盒
   */
  void setCloudBox(const Box& box) { box_ = box; }

  /**
   * @brief 设置点云文件信息
   */
  void setFileInfo(const QFileInfo& info) { file_info_ = info; }

  /**
   * @brief 设置点云点大小
   */
  void setPointSize(int point_size) { point_size_ = point_size; }

  /**
   * @brief 设置点云点颜色 (rgb)
   */
  void setCloudColor(int r, int g, int b);

  /**
   * @brief 设置点云点颜色 (x,y,z)
   */
  void setCloudColor(const QString& aixs);

  /**
   * @brief 设置包围盒颜色
   */
  void setBoxColor(int r, int g, int b) { box_rgb_ = pcl::RGB(r, g, b); }

  /**
   * @brief 设置法线颜色
   */
  void setNormalColor(int r, int g, int b) { normals_rgb_ = pcl::RGB(r, g, b); }

  /**
   * @brief 设置点云透明度
   */
  void setOpacity(float opacity) { opacity_ = opacity; }

  /**
   * @brief 按照维度(x,y,z)缩放点云尺寸
   * @param origin  是否以坐标原点为缩放中心
   */
  void scale(double x, double y, double z, bool origin = false);

  /**
   * @brief 更新点云
   * @param compute_resolution 是否计算分辨率
   */
  void update(bool compute_resolution = true);

 private:
  Box box_;
  QString id_;
  QString type_;
  QFileInfo file_info_;
  pcl::RGB box_rgb_;
  pcl::RGB normals_rgb_;
  int point_size_;
  float opacity_;
  float resolution_;
  bool has_normals_;
};
}  // namespace ct
#endif  // CT_BASE_CLOUD_H