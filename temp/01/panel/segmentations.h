#ifndef SEGMENTATIONS_H
#define SEGMENTATIONS_H

#include <QDockWidget>
#include <QThread>
#include "common/cloudtree.h"
#include "modules/segmentation.h"

namespace Ui {
class Segmentations;
}

class Segmentations : public QDockWidget
{
    Q_OBJECT

public:
    explicit Segmentations(QWidget *parent = nullptr);
    ~Segmentations();
     void init(Console* &co,CloudView* &cv,CloudTree* &ct);
     void preview();
     void add();
     void apply();
     void reset();
signals:
     void SACSegmentation(const Cloud::Ptr &cloud, ModelCoefficients::Ptr &cofes,int model_type, int method_type,double threshold,
                                                    int max_iterations,double min_radius,double max_radius,bool optimize,bool negative);
     void SACSegmentationFromNormals(const Cloud::Ptr &cloud, ModelCoefficients::Ptr &cofes,int model_type, int method_type,double threshold,
                                                               int max_iterations, double min_radius,double max_radius,bool optimize,bool negative,
                                                               double distance_weightconst);
     void EuclideanClusterExtraction(const Cloud::Ptr &cloud,double tolerance,int min_cluster_size,
                                                               int max_cluster_size);
     void RegionGrowing(const Cloud::Ptr &cloud,int min_cluster_size,int max_cluster_size,
                                                  bool smoothMode,bool CurvatureTest,bool ResidualTest,float SmoothnessThreshold,
                                                  float ResidualThreshold,float CurvatureThreshold,int NumberOfNeighbours);
     void RegionGrowingRGB(const Cloud::Ptr &cloud, int min_cluster_size, int max_cluster_size, bool smoothMode,
                                                     bool CurvatureTest, bool ResidualTest, float SmoothnessThreshold, float ResidualThreshold,
                                                     float CurvatureThreshold, int NumberOfNeighbours,float pointcolorthresh,float regioncolorthresh,
                                                     float distancethresh,int nghbr_number);
     void MinCutSegmentation(const Cloud::Ptr &cloud,const Eigen::Vector3f &center,double sigma,double radius,
                                                       double weight,int neighbour_number);
     void DonSegmentation(const Cloud::Ptr &cloud,double mean_radius,double scale1,double scale2,double threshold,
                                                    double segradius,int minClusterSize ,int maxClusterSize);
     void MorphologicalFilter(const Cloud::Ptr &cloud,int max_window_size,float slope,float max_distance,float initial_distance,
                                                        float cell_size,float base,bool negative);
     void SupervoxelClustering(const Cloud::Ptr &cloud,float voxel_resolution,float seed_resolution,float color_importance,
                                                         float spatial_importance,float normal_importance, bool camera_transform);
public slots:
     void result(const std::vector<Cloud::Ptr> &cloud,float time);
protected:
    void closeEvent(QCloseEvent *event);
private:
    Ui::Segmentations *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    QThread thread;
    ModelCoefficients::Ptr coefs;
    std::vector<Cloud::Ptr> segmented_clouds;
};

#endif // SEGMENTATIONS_H
