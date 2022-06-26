/**
 * @file segmentation.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#ifndef CT_TOOL_SEGMENTATION_H
#define CT_TOOL_SEGMENTATION_H

#include "base/customdock.h"

#include "modules/segmentation.h"

#include <QThread>


namespace Ui
{
    class Segmentation;
}

class Segmentation : public ct::CustomDock
{
    Q_OBJECT

public:
    explicit Segmentation(QWidget* parent = nullptr);
    ~Segmentation();

    void preview();
    void add();
    void apply();
    virtual void reset();

signals:
    void SACSegmentation(int model, int method, double threshold, int max_iterations, double probability, bool optimize,
                         double min_radius, double max_radius);
    void SACSegmentationFromNormals(int model, int method, double threshold, int max_iterations, double probability,
                                    bool optimize, double min_radius, double max_radius,double distance_weight, double d);
    void EuclideanClusterExtraction(double tolerance, int min_cluster_size, int max_cluster_size);
    void RegionGrowing(int min_cluster_size, int max_cluster_size, bool smooth_mode, bool curvature_test, bool residual_test,
                       float smoothness_threshold, float residual_threshold, float curvature_threshold, int neighbours);
    void RegionGrowingRGB(int min_cluster_size, int max_cluster_size, bool smooth_mode, bool curvature_test,
                          bool residual_test, float smoothness_threshold, float residual_threshold, float curvature_threshold,
                          int neighbours, float pt_thresh, float re_thresh, float dis_thresh, int nghbr_number);
    void SupervoxelClustering(float voxel_resolution, float seed_resolution, float color_importance, float spatial_importance,
                              float normal_importance, bool camera_transform);

public slots:
    void segmentationResult(const QString& id, const std::vector<ct::Cloud::Ptr>& cloud, float time, const ct::ModelCoefficients::Ptr& cofe = nullptr);

private:
    Ui::Segmentation* ui;
    QThread m_thread;
    ct::Segmentation* m_seg;
    std::unordered_map<QString, std::vector<ct::Cloud::Ptr>> m_segmentation_map;

};

#endif // SEGMENTATIONS_H
