/**
 * @file cloud.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CLOUD_H
#define CT_BASE_CLOUD_H

#include "base/exports.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>

#include <QFileInfo>
#include <QString>

#define CLOUD_TYPE_XYZ          "XYZ"
#define CLOUD_TYPE_XYZRGB       "XYZRGB"
#define CLOUD_TYPE_XYZN         "XYZNormal"
#define CLOUD_TYPE_XYZRGBN      "XYZRGBNormal"

#define BOX_PRE_FLAG                "-box"
#define NORMALS_PRE_FLAG            "-normals"

namespace ct
{
    typedef pcl::Indices Indices;
    typedef pcl::console::TicToc TicToc;
    typedef pcl::PointXYZRGBNormal PointXYZRGBN;

    struct Box
    {
        double width;
        double height;
        double depth;
        Eigen::Affine3f pose;
        Eigen::Vector3f translation;
        Eigen::Quaternionf rotation;
    };

    struct RGB
    {
        RGB() {}
        RGB(int r_, int g_, int b_) :r(r_), g(g_), b(b_) {}
        double rf() const { return (double)r / 255; }
        double gf() const { return (double)g / 255; }
        double bf() const { return (double)b / 255; }
        int r;
        int g;
        int b;
    };

    // define color
    namespace Color
    {
        const RGB White = { 255,255,255 };
        const RGB Black = { 0  ,  0,  0 };
        const RGB Red = { 255,0,  0 };
        const RGB Green = { 0,  255,0 };
        const RGB Blue = { 0,  0,  255 };
        const RGB Yellow = { 255,255,0 };
        const RGB Cyan = { 0,255,255 };
        const RGB Purple = { 255,0,255 };
    }


    class CT_EXPORT Cloud : public pcl::PointCloud<PointXYZRGBN>
    {
    public:
        Cloud() : m_id("cloud"),
            m_box_rgb(Color::White),
            m_normals_rgb(Color::White),
            m_type(CLOUD_TYPE_XYZ),
            m_point_size(1),
            m_opacity(1.0),
            m_resolution(0.0)
        {}

        Cloud(const Cloud& cloud, const Indices& indices)
            : pcl::PointCloud<PointXYZRGBN>(cloud, indices) {}

        Cloud& operator+=(const Cloud& rhs)
        {
            concatenate((*this), rhs);
            return (*this);
        }

        Cloud operator+(const Cloud& rhs) { return (Cloud(*this) += rhs); }

        using Ptr = std::shared_ptr<Cloud>;
        using ConstPtr = std::shared_ptr<const Cloud>;

        Ptr makeShared() const { return Ptr(new Cloud(*this)); }

        /**
         * @brief ???????????????
         */
        Box box() const { return m_box; }

        /**
         * @brief ??????ID
         */
        QString id() const { return m_id; }

        /**
         * @brief ????????????ID
         */
        QString normalId() const { return m_id + NORMALS_PRE_FLAG; }

        /**
         * @brief ???????????????ID
         */
        QString boxId() const { return m_id + BOX_PRE_FLAG; }

        /**
         * @brief ?????????????????????
         */
        RGB boxColor() const { return m_box_rgb; }

        /**
         * @brief ??????????????????
         */
        RGB normalColor() const { return m_normals_rgb; }

        /**
         * @brief ????????????
         */
        Eigen::Vector3f center() const { return m_box.translation; }

        /**
         * @brief ????????????
         */
        QString type() const { return m_type; }

        /**
         * @brief ??????????????????
         */
        QFileInfo info() const { return m_info; }

        /**
         * @brief ??????????????????
         */
        QString path() const { return m_info.path(); }

        /**
         * @brief ???????????????
         */
        PointXYZRGBN min() const { return m_min; }

        /**
         * @brief ???????????????
         */
        PointXYZRGBN max() const { return m_max; }

        /**
         * @brief ?????????????????????KB)
         */
        int fileSize() const { return m_info.size() / 1024; }

        /**
         * @brief ???????????????
         */
        int pointSize() const { return m_point_size; }

        /**
         * @brief ???????????????
         */
        float opacity() const { return m_opacity; }

        /**
         * @brief ???????????????
         */
        float resolution() const { return m_resolution; }

        /**
         * @brief ????????????
         */
        float volume() const { return m_box.depth * m_box.height * m_box.width; }

        /**
         * @brief ?????????????????????
         */
        bool hasNormals() const { return points[rand() % size()].normal_x != 0.0f; }

        /**
         * @brief ????????????ID
         */
        void setId(const QString& id) { m_id = id; }

        /**
         * @brief ?????????????????????
         */
        void setBox(const Box& box) { m_box = box; }

        /**
         * @brief ????????????????????????
         */
        void setInfo(const QFileInfo& info) { m_info = info; }

        /**
         * @brief ?????????????????????
         */
        void setPointSize(int point_size) { m_point_size = point_size; }

        /**
         * @brief ????????????????????? (rgb)
         */
        void setCloudColor(const RGB& rgb);

        /**
         * @brief ????????????????????? (aixs)
         */
        void setCloudColor(const QString& aixs);

        /**
         * @brief ?????????????????????
         */
        void setBoxColor(const RGB& rgb) { m_box_rgb = rgb; }

        /**
         * @brief ??????????????????
         */
        void setNormalColor(const RGB& rgb) { m_normals_rgb = rgb; }

        /**
         * @brief ?????????????????????
         */
        void setOpacity(float opacity) { m_opacity = opacity; }

        /**
         * @brief ????????????(x,y,z)??????????????????
         * @param origin  ????????????????????????????????????
         */
        void scale(double x, double y, double z, bool origin = false);

        /**
         * @brief ????????????
         * @param resolution_flag ?????????????????????
         * @param box_flag ?????????????????????
         * @param type_flag ????????????????????????
         */
        void update(bool box_flag = true, bool type_flag = true, bool resolution_flag = true);

    private:
        Box m_box;
        QString m_id;
        RGB m_box_rgb;
        RGB m_normals_rgb;
        QString m_type;
        QFileInfo m_info;
        int m_point_size;
        float m_opacity;
        float m_resolution;
        PointXYZRGBN m_min;
        PointXYZRGBN m_max;
    };
} // namespace ct
#endif // CT_BASE_CLOUD_H