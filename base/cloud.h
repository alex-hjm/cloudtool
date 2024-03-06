/**
 * @file cloud.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-28
 */
#ifndef __BASE_CLOUD_H__
#define __BASE_CLOUD_H__

#include "types.h"

#include <pcl/point_cloud.h>
#include <QDataStream>
#include <QString>
#include <QMetaType>

#define CLOUD_DEFAULT_ID    "undefined"
#define CLOUD_BBOX_SUFFIX   "-bbox"

CT_BEGIN_NAMESPACE

class CT_EXPORT Cloud : public pcl::PointCloud<PointXYZRGBN>
{
public:
    Cloud() : m_id (CLOUD_DEFAULT_ID), 
              m_bbox_rgb(Color::Black), 
              m_point_size(1),
              m_opacity(1.0),
              m_resolution(1.0) {}

    Cloud(const QString& id) : Cloud() { m_id = id; }

    Cloud& operator+=(const Cloud& rhs) { concatenate((*this), rhs); return (*this); }
    Cloud operator+(const Cloud& rhs) { return (Cloud(*this) += rhs); }

    using Ptr = std::shared_ptr<Cloud>;
    using ConstPtr = std::shared_ptr<const Cloud>;
    Ptr makeShared() const { return Ptr(new Cloud(*this)); }

    QString id() const { return m_id; }
    QString bboxId() const { return id() + CLOUD_BBOX_SUFFIX; }
    QString path() const { return m_path; }
    BBox bbox() const { return m_bbox; }
    RGB bboxColor() const { return m_bbox_rgb; }
    int pointSize() const { return m_point_size; }
    float opacity() const { return m_opacity; }
    int pointNum() const { return this->size(); }
    float resolution() const { return m_resolution; }

    void setId(const QString& id) { m_id = id; }
    void setPath(const QString& path) { m_path = path; }
    void setBBox(const BBox& bbox) { m_bbox = bbox; }
    void setBBoxColor(const RGB& rgb) { m_bbox_rgb = rgb; }
    void setPointSize(int size) { m_point_size = size; }
    void setOpacity(float opacity) { m_opacity = opacity; }

    void setColor(const RGB& rgb);
    void updateBBox();
    void updateResolution();

    friend QDataStream &operator<<(QDataStream &out, Cloud::Ptr const &rhs) {
        out.writeRawData(reinterpret_cast<const char*>(&rhs), sizeof(rhs));
        return out;
    }

    friend QDataStream & operator >> (QDataStream &in, Cloud::Ptr &rhs) {
        in.readRawData(reinterpret_cast<char*>(&rhs), sizeof(rhs));
        return in;
    }

private:
    QString m_id;
    QString m_path;
    BBox m_bbox;
    RGB m_bbox_rgb;
    int m_point_size;
    float m_opacity;
    float m_resolution;
};

CT_END_NAMESPACE

Q_DECLARE_METATYPE(CT_NAMESPACE::Cloud::Ptr)
Q_DECLARE_METATYPE(CT_NAMESPACE::Coord)

#endif  // __BASE_CLOUD_H__