/**
 * @file descriptor.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_TOOL_DESCRIPTOR_H
#define CT_TOOL_DESCRIPTOR_H

#include "base/customdock.h"

#include "modules/features.h"

#include <pcl/visualization/pcl_plotter.h>

#include <QThread>

namespace Ui
{
    class Descriptor;
}

class Descriptor : public ct::CustomDock
{
    Q_OBJECT

public:
    explicit Descriptor(QWidget* parent = nullptr);
    ~Descriptor();

    void preview();
    virtual void reset();

    ct::FeatureType::Ptr getDescriptor(const QString& id)
    {
        if (m_descriptor_map.find(id) == m_descriptor_map.end())
            return nullptr;
        else
            return m_descriptor_map.find(id)->second;
    }

signals:
    void PFHEstimation();
    void FPFHEstimation();
    void VFHEstimation(const Eigen::Vector3f& dir);
    void ESFEstimation();
    void GASDEstimation(const Eigen::Vector3f& dir, int shgs, int shs, int interp);
    void GASDColorEstimation(const Eigen::Vector3f& dir, int shgs, int shs, int interp,
                             int chgs, int chs, int cinterp);
    void RSDEstimation(int nr_subdiv, double plane_radius);
    void GRSDEstimation();
    void CRHEstimation(const Eigen::Vector3f& dir);
    void CVFHEstimation(const Eigen::Vector3f& dir, float radius_normals,
                        float d1, float d2, float d3, int min, bool normalize);
    void ShapeContext3DEstimation(double min_radius, double radius);
    void SHOTEstimation(const ct::ReferenceFrame::Ptr& lrf, float radius);
    void SHOTColorEstimation(const ct::ReferenceFrame::Ptr& lrf, float radius);
    void UniqueShapeContext(const ct::ReferenceFrame::Ptr& lrf, double min_radius, double pt_radius, double loc_radius);
    void BOARDLocalReferenceFrameEstimation(float radius, bool find_holes, float margin_thresh, int size,
                                            float prob_thresh, float steep_thresh);
    void FLARELocalReferenceFrameEstimation(float radius, float margin_thresh, int min_neighbors_for_normal_axis,
                                            int min_neighbors_for_tangent_axis);
    void SHOTLocalReferenceFrameEstimation();


public slots:
    void featureResult(const QString& id, const ct::FeatureType::Ptr& feature, float time);
    void lrfResult(const QString& id, const ct::ReferenceFrame::Ptr& cloud, float time);

private:
    Ui::Descriptor* ui;
    QThread m_thread;
    ct::Features* m_features;
    pcl::visualization::PCLPlotter::Ptr m_plotter;
    std::map<QString, ct::FeatureType::Ptr> m_descriptor_map;
    std::map<QString, ct::ReferenceFrame::Ptr> m_lrf_map;

};

#endif // DESCRIPTOR_H
