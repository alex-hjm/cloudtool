#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <QObject>
#include <pcl/console/time.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>
#include "common/cloud.h"

class Registration : public QObject
{
    Q_OBJECT
public:
    explicit Registration(QObject *parent = nullptr);

signals:
    void result(bool success,const Cloud::Ptr &ail_cloud,const Eigen::Matrix4f &matrix,double score,float time);
public slots:
    void IterativeClosestPoint(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                               double distance_threshold,double euclidean_fitness_epsilon, bool use_reciprocal_correspondence);
    void IterativeClosestPointWithNormals(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                          double distance_threshold, double euclidean_fitness_epsilon,bool use_reciprocal_correspondence,bool use_symmetric_objective,
                                          bool enforce_same_direction_normals);
    void IterativeClosestPointNonLinear(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                        double distance_threshold,double euclidean_fitness_epsilon, bool use_reciprocal_correspondence);
    void NormalDistributionsTransform(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                      double distance_threshold,double euclidean_fitness_epsilon, float resolution,double step_size,double outlier_ratio);
    void SampleConsensusPrerejectiveFPFH(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,const FPFHFeature::Ptr &targetFeature,const FPFHFeature::Ptr &sourceFeature,
                                     int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                     int nr_samples, int k, float similarity_threshold,float inlier_fraction);
    void SampleConsensusPrerejectiveSHOT(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,const SHOTFeature::Ptr &targetFeature,const SHOTFeature::Ptr &sourceFeature,
                                     int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                     int nr_samples, int k, float similarity_threshold,float inlier_fraction);
    void SampleConsensusInitialAlignmentFPFH(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,const FPFHFeature::Ptr &targetFeature,const FPFHFeature::Ptr &sourceFeature,
                                         int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                         int nr_samples, int k,float min_sample_distance);
    void SampleConsensusInitialAlignmentSHOT(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,const SHOTFeature::Ptr &targetFeature,const SHOTFeature::Ptr &sourceFeature,
                                         int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                         int nr_samples, int k,float min_sample_distance);
    void FPCSInitialAlignment(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold
                              ,double euclidean_fitness_epsilon, float delta,float approx_overlap,float score_threshold,int nr_samples);
    void KFPCSInitialAlignment(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,
                               double euclidean_fitness_epsilon, float delta,float approx_overlap,float score_threshold,int nr_samples,float upper_trl_boundary,float lower_trl_boundary,
                               float lambda);

};

#endif // REGISTRATION_H
