#ifndef REGISTRATIONS_H
#define REGISTRATIONS_H

#include <QDockWidget>
#include "common/cloudtree.h"
#include "common/tool.h"
#include "modules/registration.h"

namespace Ui {
class Registrations;
}

class Registrations : public QDockWidget
{
    Q_OBJECT

public:
    explicit Registrations(QWidget *parent = nullptr);
    ~Registrations();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void setTarget();
    void setSource();
    void preview();
    void add();
    void apply();
    void reset();

signals:
    void IterativeClosestPoint(const Cloud::Ptr &target_cloud,const Cloud::Ptr &source_cloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                               double distance_threshold,double euclidean_fitness_epsilon, bool use_reciprocal_correspondence);
    void IterativeClosestPointWithNormals(const Cloud::Ptr &target_cloud,const Cloud::Ptr &source_cloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                          double distance_threshold, double euclidean_fitness_epsilon,bool use_reciprocal_correspondence,bool use_symmetric_objective,
                                          bool enforce_same_direction_normals);
    void IterativeClosestPointNonLinear(const Cloud::Ptr &target_cloud,const Cloud::Ptr &source_cloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                        double distance_threshold,double euclidean_fitness_epsilon, bool use_reciprocal_correspondence);
    void NormalDistributionsTransform(const Cloud::Ptr &target_cloud,const Cloud::Ptr &source_cloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                      double distance_threshold,double euclidean_fitness_epsilon, float resolution,double step_size,double outlier_ratio);
    void SampleConsensusPrerejectiveFPFH(const Cloud::Ptr &target_cloud,const Cloud::Ptr &source_cloud,const FPFHFeature::Ptr &targetFeature,const FPFHFeature::Ptr &sourceFeature,
                                     int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                     int nr_samples, int k, float similarity_threshold,float inlier_fraction);
    void SampleConsensusPrerejectiveSHOT(const Cloud::Ptr &target_cloud,const Cloud::Ptr &source_cloud,const SHOTFeature::Ptr &targetFeature,const SHOTFeature::Ptr &sourceFeature,
                                     int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                     int nr_samples, int k, float similarity_threshold,float inlier_fraction);
    void SampleConsensusInitialAlignmentFPFH(const Cloud::Ptr &target_cloud,const Cloud::Ptr &source_cloud,const FPFHFeature::Ptr &targetFeature,const FPFHFeature::Ptr &sourceFeature,
                                         int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                         int nr_samples, int k,float min_sample_distance);
    void SampleConsensusInitialAlignmentSHOT(const Cloud::Ptr &target_cloud,const Cloud::Ptr &source_cloud,const SHOTFeature::Ptr &targetFeature,const SHOTFeature::Ptr &sourceFeature,
                                         int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                         int nr_samples, int k,float min_sample_distance);
    void FPCSInitialAlignment(const Cloud::Ptr &target_cloud,const Cloud::Ptr &source_cloud,int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold
                              ,double euclidean_fitness_epsilon, float delta,float approx_overlap,float score_threshold,int nr_samples);
    void KFPCSInitialAlignment(const Cloud::Ptr &target_cloud,const Cloud::Ptr &source_cloud,int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,
                               double euclidean_fitness_epsilon, float delta,float approx_overlap,float score_threshold,int nr_samples,float upper_trl_boundary,float lower_trl_boundary,
                               float lambda);

public slots:
    void result(bool success,const Cloud::Ptr &cloud,const Eigen::Matrix4f &matrix,double score,float time);
protected:
    void closeEvent(QCloseEvent *event);
private:
    Ui::Registrations *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    QThread thread;
    int num_iterations;
    Index target_index;
    Cloud::Ptr target_cloud;
    Index source_index;
    Cloud::Ptr source_cloud;
    Cloud::Ptr ail_cloud;
};

#endif // REGISTRATIONS_H
