#ifndef REGISTRATIONS_H
#define REGISTRATIONS_H

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_organized_boundary.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>

#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_symmetric_point_to_plane_lls.h>
#include "src/common/customtype.h"

typedef pcl::registration::CorrespondenceEstimationBase<PointXYZRGBN,PointXYZRGBN>::Ptr CorrespondenceEstimationPtr;
typedef pcl::registration::TransformationEstimation<PointXYZRGBN, PointXYZRGBN>::Ptr TransformationEstimationPtr;
typedef pcl::registration::CorrespondenceRejector::Ptr CorrespondenceRejectorPtr;

class Registrations
{
public:
    Registrations();

    float tocTime;
    double score;
    Eigen::Matrix4f matrix;
    CloudXYZRGBN::Ptr IterativeClosestPoint(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,
                                            int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, bool use_reciprocal_correspondence);
    CloudXYZRGBN::Ptr IterativeClosestPointWithNormals(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,
                                                       int ransac_iterations,double inlier_threshold,double distance_threshold, double euclidean_fitness_epsilon,
                                                       bool use_reciprocal_correspondence,bool use_symmetric_objective,bool enforce_same_direction_normals);
    CloudXYZRGBN::Ptr IterativeClosestPointNonLinear(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,
                                                     int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, bool use_reciprocal_correspondence);
    CloudXYZRGBN::Ptr FPCSInitialAlignment(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,
                                           double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, float delta,float approx_overlap,float score_threshold,int nr_samples);
    CloudXYZRGBN::Ptr KFPCSInitialAlignment(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,
                                            double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, float delta,float approx_overlap,float score_threshold,int nr_samples,
                                            float upper_trl_boundary,float lower_trl_boundary,float lambda);
    CloudXYZRGBN::Ptr SampleConsensusPrerejective(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const FPFHFeature::Ptr &targetFeature,
                                                  const FPFHFeature::Ptr &sourceFeature,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                                  double distance_threshold,double euclidean_fitness_epsilon,int nr_samples, int k, float similarity_threshold,float inlier_fraction);
    CloudXYZRGBN::Ptr SampleConsensusPrerejective(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const SHOTFeature::Ptr &targetFeature,
                                                  const SHOTFeature::Ptr &sourceFeature,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                                  double distance_threshold,double euclidean_fitness_epsilon,int nr_samples, int k, float similarity_threshold,float inlier_fraction);
    CloudXYZRGBN::Ptr SampleConsensusInitialAlignment(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const FPFHFeature::Ptr &targetFeature,
                                                      const FPFHFeature::Ptr &sourceFeature,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                                      double distance_threshold,double euclidean_fitness_epsilon,int nr_samples, int k,float min_sample_distance);
    CloudXYZRGBN::Ptr SampleConsensusInitialAlignment(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const SHOTFeature::Ptr &targetFeature,
                                                      const SHOTFeature::Ptr &sourceFeature,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                                      double distance_threshold,double euclidean_fitness_epsilon,int nr_samples, int k,float min_sample_distance);
    CloudXYZRGBN::Ptr NormalDistributionsTransform(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,
                                                   double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, float resolution,double step_size,double outlier_ratio);
    //Correspondence Estimation Method
    CorrespondencesPtr CorrespondenceEstimation(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud);
    CorrespondencesPtr CorrespondenceEstimationBackprojection(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int k);
    CorrespondencesPtr CorrespondenceEstimationNormalShooting(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int k);

    CorrespondencesPtr CorrespondenceEstimation(const FPFHFeature::Ptr &targetFeature,const FPFHFeature::Ptr &sourceFeature);
    CorrespondencesPtr CorrespondenceEstimation(const SHOTFeature::Ptr &targetFeature,const SHOTFeature::Ptr &sourceFeature);
    CorrespondencesPtr CorrespondenceEstimationFPFHKDTree(const FPFHFeature::Ptr &targetFeature,const FPFHFeature::Ptr &sourceFeature,double threshold);
    CorrespondencesPtr CorrespondenceEstimationSHOTKDTree(const SHOTFeature::Ptr &targetFeature,const SHOTFeature::Ptr &sourceFeature,double threshold);

    //Transformation Estimation Method
    TransformationEstimationPtr TransformationEstimationLM();
    TransformationEstimationPtr TransformationEstimationPointToPlane();
    TransformationEstimationPtr TransformationEstimationPointToPlaneLLS();
    TransformationEstimationPtr TransformationEstimationPointToPlaneLLSWeighted();
    TransformationEstimationPtr TransformationEstimationSVD ();
    TransformationEstimationPtr TransformationEstimationSVDScale();
    TransformationEstimationPtr TransformationEstimationSymmetricPointToPlaneLLS ();

    //Correspondence Rejection Method
    CorrespondencesPtr CorrespondenceRejectorDistance(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const CorrespondencesPtr &original_correspondences,
                                                      float max_distance);
    CorrespondencesPtr CorrespondenceRejectorMedianDistance(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const CorrespondencesPtr &original_correspondences,
                                                            double factor);
    CorrespondencesPtr CorrespondenceRejectorOneToOne(const CorrespondencesPtr &original_correspondences);
    CorrespondencesPtr CorrespondenceRejectionOrganizedBoundary(const CorrespondencesPtr &original_correspondences);
    CorrespondencesPtr CorrespondenceRejectorPoly(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const CorrespondencesPtr &original_correspondences,int cardinality,
                                                  float similarity_threshold,int iterations);
    CorrespondencesPtr CorrespondenceRejectorSampleConsensus(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const CorrespondencesPtr &original_correspondences,
                                                             double threshold,int max_iterations,const bool refine,bool s);
    CorrespondencesPtr CorrespondenceRejectorSurfaceNormal(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const CorrespondencesPtr &original_correspondences,
                                                           double threshold);
    CorrespondencesPtr CorrespondenceRejectorTrimmed(const CorrespondencesPtr &original_correspondences,float ratio,int min_correspondences);
    CorrespondencesPtr CorrespondenceRejectorVarTrimmed(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const CorrespondencesPtr &original_correspondences,
                                                       float min_ratio,double max_ratio);

public:
    pcl::registration::CorrespondenceEstimation<PointXYZRGBN,PointXYZRGBN>::Ptr ce;
    pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBN,PointXYZRGBN,PointXYZRGBN>::Ptr cebp;
    pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBN,PointXYZRGBN,PointXYZRGBN>::Ptr cens;

    pcl::registration::CorrespondenceRejectorDistance::Ptr crd;
    pcl::registration::CorrespondenceRejectorMedianDistance::Ptr crmd;
    pcl::registration::CorrespondenceRejectorOneToOne::Ptr croto;
    pcl::registration::CorrespondenceRejectionOrganizedBoundary::Ptr crob;
    pcl::registration::CorrespondenceRejectorPoly<PointXYZRGBN,PointXYZRGBN>::Ptr crp;
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBN>::Ptr crsc;
    pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr crsn;
    pcl::registration::CorrespondenceRejectorTrimmed::Ptr crt;
    pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr crvt;

private:
    pcl::console::TicToc time;
    CorrespondenceEstimationPtr getCorrespondenceEstimation(int ce_type);
    TransformationEstimationPtr getTransformationEstimation(int te_type);
    CorrespondenceRejectorPtr getCorrespondenceRejector(int cr_type);
};

#endif // REGISTRATIONS_H
