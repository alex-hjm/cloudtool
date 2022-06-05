/**
 * @file cloud.h
 * @author hjm (hjmalex@163.com)
 * @version 1.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CLOUD_H
#define CT_BASE_CLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>

#include <QFileInfo>
#include <QString>

#include "base/exports.h"

#define CLOUD_DEFAULT_ID    "cloud"
#define BOX_DEFAULT_ID      "-box"
#define NORMALS_DEFAULT_ID  "-normals"

#define CLOUD_TYPE_XYZ      "XYZ"
#define CLOUD_TYPE_XYZRGB   "XYZRGB"
#define CLOUD_TYPE_XYZN     "XYZNormal"
#define CLOUD_TYPE_XYZRGBN  "XYZRGBNormal"

namespace ct
{
  typedef pcl::RGB RGB;
  typedef pcl::Indices Indices;
  typedef pcl::console::TicToc TicToc;
  typedef pcl::PointXYZRGBNormal PointXYZRGBN;

  struct Box
  {
    double              width;
    double              height;
    double              depth;
    Eigen::Affine3f     pose;
    Eigen::Vector3f     translation;
    Eigen::Quaternionf  rotation;
  };

  class CT_EXPORT Cloud : public pcl::PointCloud<PointXYZRGBN>
  {
  public:
    Cloud() : m_id(CLOUD_DEFAULT_ID),
              m_box_rgb(255, 255, 255),
              m_normals_rgb(255, 255, 255),
              m_type(CLOUD_TYPE_XYZ),
              m_point_size(1),
              m_opacity(1.0),
              m_resolution(0.0) {}

    Cloud(const Cloud &cloud, const Indices &indices)
        : pcl::PointCloud<PointXYZRGBN>(cloud, indices) {}

    Cloud &operator+=(const Cloud &rhs)
    {
      concatenate((*this), rhs);
      return (*this);
    }

    Cloud operator+(const Cloud &rhs) { return (Cloud(*this) += rhs); }

    using Ptr = std::shared_ptr<Cloud>;
    using ConstPtr = std::shared_ptr<const Cloud>;

    Ptr makeShared() const { return Ptr(new Cloud(*this)); }

    /**
     * @brief 点云包围盒
     */
    Box box() const { return m_box; }

    /**
     * @brief 点云ID
     */
    QString id() const { return m_id; }

    /**
     * @brief 点云法线ID
     */
    QString normalId() const { return m_id + NORMALS_DEFAULT_ID; }

    /**
     * @brief 点云包围盒ID
     */
    QString boxId() const { return m_id + BOX_DEFAULT_ID; }

    /**
     * @brief 点云包围盒颜色
     */
    RGB boxColor() const { return m_box_rgb; }

    /**
     * @brief 点云法线颜色
     */
    RGB normalColor() const { return m_normals_rgb; }

    /**
     * @brief 点云中心
     */
    Eigen::Vector3f center() const { return m_box.translation; }

    /**
     * @brief 点云类型
     */
    QString type() const { return m_type; }

    /**
     * @brief 点云文件信息
     */
    QFileInfo info() const { return m_info; }

    /**
     * @brief 点云文件路径
     */
    QString path() const { return m_info.path(); }

    /**
     * @brief 点云文件大小（KB)
     */
    int fileSize() const { return m_info.size() / 1024; }

    /**
     * @brief 点云点大小
     */
    int pointSize() const { return m_point_size; }

    /**
     * @brief 点云透明度
     */
    float opacity() const { return m_opacity; }

    /**
     * @brief 点云分辨率
     */
    float resolution() const { return m_resolution; }

    /**
     * @brief 点云体积
     */
    float volume() const { return m_box.depth * m_box.height * m_box.width; }

    /**
     * @brief 点云是否有法线
     */
    bool hasNormals() const { return points[rand() % size()].normal_x != 0.0f; }

    /**
     * @brief 设置点云ID
     */
    void setId(const QString &id) { m_id = id; }

    /**
     * @brief 设置点云包围盒
     */
    void setBox(const Box &box) { m_box = box; }

    /**
     * @brief 设置点云文件信息
     */
    void setInfo(const QFileInfo &info) { m_info = info; }

    /**
     * @brief 设置点云点大小
     */
    void setPointSize(int point_size) { m_point_size = point_size; }

    /**
     * @brief 设置点云点颜色 (rgb)
     */
    void setCloudColor(int r, int g, int b);

    /**
     * @brief 设置点云点颜色 (aixs)
     */
    void setCloudColor(const QString &aixs);

    /**
     * @brief 设置包围盒颜色
     */
    void setBoxColor(int r, int g, int b) { m_box_rgb = RGB(r, g, b); }

    /**
     * @brief 设置法线颜色
     */
    void setNormalColor(int r, int g, int b) { m_normals_rgb = RGB(r, g, b); }

    /**
     * @brief 设置点云透明度
     */
    void setOpacity(float opacity) { m_opacity = opacity; }

    /**
     * @brief 按照维度(x,y,z)缩放点云尺寸
     * @param origin  是否以坐标原点为缩放中心
     */
    void scale(double x, double y, double z, bool origin = false);

    /**
     * @brief 更新点云
     * @param resolution_flag 是否更新分辨率
     * @param box_flag 是否更新包围盒
     * @param type_flag 是否更新点云类型
     */
    void update(bool resolution_flag = true, bool box_flag = true, bool type_flag = true);

  private:
    Box       m_box;
    QString   m_id;
    RGB       m_box_rgb;
    RGB       m_normals_rgb;
    QString   m_type;
    QFileInfo m_info;
    int       m_point_size;
    float     m_opacity;
    float     m_resolution;
  };
} // namespace ct
#endif // CT_BASE_CLOUD_H