#include "pca/registration.h"

#include <pcl/console/time.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_organized_boundary.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_sample_consensus_2d.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_sorting.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_3point.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_symmetric_point_to_plane_lls.h>
#include <pcl/registration/transformation_validation.h>
#include <pcl/registration/transformation_validation_euclidean.h>

namespace pca {

void Registration::CorrespondenceEstimationBackProjection(int k) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

  pcl::registration::CorrespondenceEstimationBackProjection<
      PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>
      cebp;
  cebp.setInputTarget(target_cloud_);
  cebp.setInputSource(source_cloud_);
  cebp.setSourceNormals(source_cloud_);
  cebp.setTargetNormals(target_cloud_);
  cebp.setSearchMethodTarget(target_tree);
  cebp.setSearchMethodSource(source_tree);
  cebp.setKSearch(k);
  cebp.determineReciprocalCorrespondences(*corr);
  emit correspondenceEstimationResult(corr, time.toc());
}

void Registration::CorrespondenceEstimationNormalShooting(int k) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

  pcl::registration::CorrespondenceEstimationNormalShooting<
      PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>
      cebp;
  cebp.setInputTarget(target_cloud_);
  cebp.setInputSource(source_cloud_);
  cebp.setSourceNormals(source_cloud_);
  // cebp.setTargetNormals(target_cloud_);
  cebp.setSearchMethodTarget(target_tree);
  cebp.setSearchMethodSource(source_tree);
  cebp.setKSearch(k);
  cebp.determineReciprocalCorrespondences(*corr);
  emit correspondenceEstimationResult(corr, time.toc());
}

void Registration::CorrespondenceEstimationOrganizedProjection(
    float fx, float fy, float cx, float cy,
    const Eigen::Matrix4f &src_to_tgt_trans, float depth_threshold) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

  pcl::registration::CorrespondenceEstimationOrganizedProjection<PointXYZRGBN,
                                                                 PointXYZRGBN>
      cebp;
  cebp.setInputTarget(target_cloud_);
  cebp.setInputSource(source_cloud_);
  // cebp.setSourceNormals(source_cloud_);
  // cebp.setTargetNormals(target_cloud_);
  cebp.setSearchMethodTarget(target_tree);
  cebp.setSearchMethodSource(source_tree);
  cebp.setFocalLengths(fx, fy);
  cebp.setCameraCenters(cx, cy);
  cebp.setSourceTransformation(src_to_tgt_trans);
  cebp.setDepthThreshold(depth_threshold);
  double max_distance = 0.0;
  cebp.determineReciprocalCorrespondences(*corr, max_distance);
  emit correspondenceEstimationResult(corr, time.toc());
}

double Registration::DataContainer(const pcl::Correspondence &corr,
                                   bool from_normals) {
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);

  pcl::registration::DataContainer<PointXYZRGBN, PointXYZRGBN> dc;
  dc.setInputTarget(target_cloud_);
  dc.setInputSource(source_cloud_);
  dc.setTargetNormals(target_cloud_);
  dc.setSearchMethodTarget(target_tree);
  dc.setInputNormals(source_cloud_);
  if (from_normals)
    return dc.getCorrespondenceScoreFromNormals(corr);
  else
    return dc.getCorrespondenceScore(corr);
}

void Registration::CorrespondenceRejectorDistance(float distance) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);

  pcl::registration::CorrespondenceRejectorDistance cj;
  cj.setInputTarget<PointXYZRGBN>(target_cloud_);
  cj.setInputSource<PointXYZRGBN>(source_cloud_);
  // cj.setSourceNormals(source_cloud_);
  // cj.setTargetNormals(target_cloud_);
  cj.setSearchMethodTarget<PointXYZRGBN>(target_tree);
  cj.setInputCorrespondences(corr);
  cj.setMaximumDistance(distance);
  cj.getCorrespondences(*corr);
  emit correspondenceRejectorResult(corr, time.toc());
}

void Registration::CorrespondenceRejectorMedianDistance(double factor) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);

  pcl::registration::CorrespondenceRejectorMedianDistance cj;
  cj.setInputTarget<PointXYZRGBN>(target_cloud_);
  cj.setInputSource<PointXYZRGBN>(source_cloud_);
  // cj.setSourceNormals(source_cloud_);
  // cj.setTargetNormals(target_cloud_);
  cj.setSearchMethodTarget<PointXYZRGBN>(target_tree);
  cj.setMedianFactor(factor);
  cj.setInputCorrespondences(corr);
  cj.getCorrespondences(*corr);
  emit correspondenceRejectorResult(corr, time.toc());
}

void Registration::CorrespondenceRejectorOneToOne() {
  pcl::console::TicToc time;
  time.tic();

  pcl::registration::CorrespondenceRejectorOneToOne cj;
  // cj.setSourceNormals(source_cloud_);
  // cj.setTargetNormals(target_cloud_);
  cj.setInputCorrespondences(corr);
  cj.getCorrespondences(*corr);
  emit correspondenceRejectorResult(corr, time.toc());
}

void Registration::CorrespondenceRejectionOrganizedBoundary(int val) {
  pcl::console::TicToc time;
  time.tic();
  pcl::registration::CorrespondenceRejectionOrganizedBoundary cj;
  cj.setInputTarget<PointXYZRGBN>(target_cloud_);
  cj.setInputSource<PointXYZRGBN>(source_cloud_);
  // cj.setSourceNormals(source_cloud_);
  // cj.setTargetNormals(target_cloud_);
  cj.setNumberOfBoundaryNaNs(val);
  cj.setInputCorrespondences(corr);
  cj.getCorrespondences(*corr);
  emit correspondenceRejectorResult(corr, time.toc());
}

void Registration::CorrespondenceRejectorPoly(int cardinality,
                                              float similarity_threshold,
                                              int iterations) {
  pcl::console::TicToc time;
  time.tic();
  pcl::registration::CorrespondenceRejectorPoly<PointXYZRGBN, PointXYZRGBN> cj;
  cj.setInputTarget(target_cloud_);
  cj.setInputSource(source_cloud_);
  // cj.setSourceNormals(source_cloud_);
  // cj.setTargetNormals(target_cloud_);
  cj.setCardinality(cardinality);
  cj.setSimilarityThreshold(similarity_threshold);
  cj.setIterations(iterations);
  cj.setInputCorrespondences(corr);
  cj.getCorrespondences(*corr);
  emit correspondenceRejectorResult(corr, time.toc());
}

void Registration::CorrespondenceRejectorSampleConsensus(double threshold,
                                                         int max_iterations,
                                                         bool refine) {
  pcl::console::TicToc time;
  time.tic();
  pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBN> cj;
  cj.setInputTarget(target_cloud_);
  cj.setInputSource(source_cloud_);
  // cj.setSourceNormals(source_cloud_);
  // cj.setTargetNormals(target_cloud_);
  cj.setInlierThreshold(threshold);
  cj.setMaximumIterations(max_iterations);
  cj.setRefineModel(refine);
  cj.setInputCorrespondences(corr);
  cj.getCorrespondences(*corr);
  emit correspondenceRejectorResult(corr, time.toc());
}

void Registration::CorrespondenceRejectorSurfaceNormal(double threshold) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::registration::CorrespondenceRejectorSurfaceNormal cj;
  cj.setInputTarget<PointXYZRGBN>(target_cloud_);
  cj.setInputSource<PointXYZRGBN>(source_cloud_);
  // cj.setSourceNormals(source_cloud_);
  // cj.setTargetNormals(target_cloud_);
  cj.setSearchMethodTarget<PointXYZRGBN>(target_tree);
  cj.setThreshold(threshold);
  cj.setInputCorrespondences(corr);
  cj.getCorrespondences(*corr);
  emit correspondenceRejectorResult(corr, time.toc());
}

void Registration::CorrespondenceRejectorTrimmed(float ratio, int min_corre) {
  pcl::console::TicToc time;
  time.tic();
  pcl::registration::CorrespondenceRejectorTrimmed cj;
  // cj.setSourceNormals(source_cloud_);
  // cj.setTargetNormals(target_cloud_);
  cj.setOverlapRatio(ratio);
  cj.setMinCorrespondences(min_corre);
  cj.setInputCorrespondences(corr);
  cj.getCorrespondences(*corr);
  emit correspondenceRejectorResult(corr, time.toc());
}

void Registration::CorrespondenceRejectorVarTrimmed(double min_ratio,
                                                    double max_ratio) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::registration::CorrespondenceRejectorVarTrimmed cj;
  cj.setInputTarget<PointXYZRGBN>(target_cloud_);
  cj.setInputSource<PointXYZRGBN>(source_cloud_);
  // cj.setSourceNormals(source_cloud_);
  // cj.setTargetNormals(target_cloud_);
  cj.setSearchMethodTarget<PointXYZRGBN>(target_tree);
  cj.setMinRatio(min_ratio);
  cj.setMaxRatio(max_ratio);
  cj.setInputCorrespondences(corr);
  cj.getCorrespondences(*corr);
  emit correspondenceRejectorResult(corr, time.toc());
}

void Registration::GeneralizedIterativeClosestPoint(int k, int max,
                                                    double tra_tolerance,
                                                    double rol_tolerance,
                                                    bool use_recip_corre) {
  pcl::console::TicToc time;
  time.tic();
  Cloud::Ptr ail_cloud(new Cloud);
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::GeneralizedIterativeClosestPoint<PointXYZRGBN, PointXYZRGBN> reg;

  reg.setInputTarget(target_cloud_);
  reg.setInputSource(source_cloud_);
  reg.setSearchMethodTarget(target_tree);
  reg.setSearchMethodSource(source_tree);
  reg.setTransformationEstimation(te_);
  reg.setCorrespondenceEstimation(ce_);
  for (auto &cj : cr_map) reg.addCorrespondenceRejector(cj.second);
  reg.setMaximumIterations(nr_iterations_);
  reg.setRANSACIterations(ransac_iterations_);
  reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
  reg.setMaxCorrespondenceDistance(distance_threshold_);
  reg.setTransformationEpsilon(transformation_epsilon_);
  reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
  reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

  reg.setCorrespondenceRandomness(k);
  reg.setMaximumOptimizerIterations(max);
  reg.setTranslationGradientTolerance(tra_tolerance);
  reg.setRotationGradientTolerance(rol_tolerance);
  reg.setUseReciprocalCorrespondences(use_recip_corre);
  reg.align(*ail_cloud);

  emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                          reg.getFinalTransformation().cast<float>(),
                          time.toc());
}

void Registration::FPCSInitialAlignment(float delta, bool normalize,
                                        float approx_overlap,
                                        float score_threshold, int nr_samples,
                                        float max_norm_diff, int max_runtime) {
  pcl::console::TicToc time;
  time.tic();
  Cloud::Ptr ail_cloud(new Cloud);
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::registration::FPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN,
                                          PointXYZRGBN>
      reg;

  reg.setInputTarget(target_cloud_);
  reg.setInputSource(source_cloud_);
  reg.setSourceNormals(source_cloud_);
  reg.setTargetNormals(target_cloud_);
  reg.setSearchMethodTarget(target_tree);
  reg.setSearchMethodSource(source_tree);
  reg.setTransformationEstimation(te_);
  reg.setCorrespondenceEstimation(ce_);
  for (auto &cj : cr_map) reg.addCorrespondenceRejector(cj.second);
  reg.setMaximumIterations(nr_iterations_);
  reg.setRANSACIterations(ransac_iterations_);
  reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
  reg.setMaxCorrespondenceDistance(distance_threshold_);
  reg.setTransformationEpsilon(transformation_epsilon_);
  reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
  reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

  reg.setDelta(delta, normalize);
  reg.setApproxOverlap(approx_overlap);
  reg.setScoreThreshold(score_threshold);
  reg.setNumberOfSamples(nr_samples);
  reg.setMaxNormalDifference(max_norm_diff);
  reg.setMaxComputationTime(max_runtime);

  reg.align(*ail_cloud);
  emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                          reg.getFinalTransformation().cast<float>(),
                          time.toc());
}

void Registration::KFPCSInitialAlignment(
    float delta, bool normalize, float approx_overlap, float score_threshold,
    int nr_samples, float max_norm_diff, int max_runtime,
    float upper_trl_boundary, float lower_trl_boundary, float lambda) {
  pcl::console::TicToc time;
  time.tic();
  Cloud::Ptr ail_cloud(new Cloud);
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::registration::KFPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN,
                                           PointXYZRGBN>
      reg;

  reg.setInputTarget(target_cloud_);
  reg.setInputSource(source_cloud_);
  reg.setSourceNormals(source_cloud_);
  reg.setTargetNormals(target_cloud_);
  reg.setSearchMethodTarget(target_tree);
  reg.setSearchMethodSource(source_tree);
  reg.setTransformationEstimation(te_);
  reg.setCorrespondenceEstimation(ce_);
  for (auto &cj : cr_map) reg.addCorrespondenceRejector(cj.second);
  reg.setMaximumIterations(nr_iterations_);
  reg.setRANSACIterations(ransac_iterations_);
  reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
  reg.setMaxCorrespondenceDistance(distance_threshold_);
  reg.setTransformationEpsilon(transformation_epsilon_);
  reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
  reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

  reg.setDelta(delta, normalize);
  reg.setApproxOverlap(approx_overlap);
  reg.setScoreThreshold(score_threshold);
  reg.setNumberOfSamples(nr_samples);
  reg.setMaxNormalDifference(max_norm_diff);
  reg.setMaxComputationTime(max_runtime);

  reg.setUpperTranslationThreshold(upper_trl_boundary);
  reg.setLowerTranslationThreshold(lower_trl_boundary);
  reg.setLambda(lambda);

  reg.align(*ail_cloud);
  emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                          reg.getFinalTransformation().cast<float>(),
                          time.toc());
}

void Registration::IterativeClosestPoint(bool use_recip_corre) {
  pcl::console::TicToc time;
  time.tic();
  Cloud::Ptr ail_cloud(new Cloud);
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::IterativeClosestPoint<PointXYZRGBN, PointXYZRGBN> reg;

  reg.setInputTarget(target_cloud_);
  reg.setInputSource(source_cloud_);
  reg.setSearchMethodTarget(target_tree);
  reg.setSearchMethodSource(source_tree);
  reg.setTransformationEstimation(te_);
  reg.setCorrespondenceEstimation(ce_);
  for (auto &cj : cr_map) reg.addCorrespondenceRejector(cj.second);
  reg.setMaximumIterations(nr_iterations_);
  reg.setRANSACIterations(ransac_iterations_);
  reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
  reg.setMaxCorrespondenceDistance(distance_threshold_);
  reg.setTransformationEpsilon(transformation_epsilon_);
  reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
  reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

  reg.setUseReciprocalCorrespondences(use_recip_corre);

  reg.align(*ail_cloud);
  emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                          reg.getFinalTransformation().cast<float>(),
                          time.toc());
}

void Registration::IterativeClosestPointWithNormals(
    bool use_recip_corre, bool use_symmetric_objective,
    bool enforce_same_direction_normals) {
  pcl::console::TicToc time;
  time.tic();
  Cloud::Ptr ail_cloud(new Cloud);
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::IterativeClosestPointWithNormals<PointXYZRGBN, PointXYZRGBN> reg;

  reg.setInputTarget(target_cloud_);
  reg.setInputSource(source_cloud_);
  reg.setSearchMethodTarget(target_tree);
  reg.setSearchMethodSource(source_tree);
  reg.setTransformationEstimation(te_);
  reg.setCorrespondenceEstimation(ce_);
  for (auto &cj : cr_map) reg.addCorrespondenceRejector(cj.second);
  reg.setMaximumIterations(nr_iterations_);
  reg.setRANSACIterations(ransac_iterations_);
  reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
  reg.setMaxCorrespondenceDistance(distance_threshold_);
  reg.setTransformationEpsilon(transformation_epsilon_);
  reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
  reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

  reg.setUseReciprocalCorrespondences(use_recip_corre);
  reg.setUseSymmetricObjective(use_symmetric_objective);
  reg.setEnforceSameDirectionNormals(enforce_same_direction_normals);

  reg.align(*ail_cloud);
  emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                          reg.getFinalTransformation().cast<float>(),
                          time.toc());
}

void Registration::IterativeClosestPointNonLinear(bool use_recip_corre) {
  pcl::console::TicToc time;
  time.tic();
  Cloud::Ptr ail_cloud(new Cloud);
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::IterativeClosestPointNonLinear<PointXYZRGBN, PointXYZRGBN> reg;

  reg.setInputTarget(target_cloud_);
  reg.setInputSource(source_cloud_);
  reg.setSearchMethodTarget(target_tree);
  reg.setSearchMethodSource(source_tree);
  reg.setTransformationEstimation(te_);
  reg.setCorrespondenceEstimation(ce_);
  for (auto &cj : cr_map) reg.addCorrespondenceRejector(cj.second);
  reg.setMaximumIterations(nr_iterations_);
  reg.setRANSACIterations(ransac_iterations_);
  reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
  reg.setMaxCorrespondenceDistance(distance_threshold_);
  reg.setTransformationEpsilon(transformation_epsilon_);
  reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
  reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

  reg.setUseReciprocalCorrespondences(use_recip_corre);

  reg.align(*ail_cloud);
  emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                          reg.getFinalTransformation().cast<float>(),
                          time.toc());
}

void Registration::NormalDistributionsTransform(float resolution,
                                                double step_size,
                                                double outlier_ratio) {
  pcl::console::TicToc time;
  time.tic();
  Cloud::Ptr ail_cloud(new Cloud);
  pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
  pcl::NormalDistributionsTransform<PointXYZRGBN, PointXYZRGBN> reg;

  reg.setInputTarget(target_cloud_);
  reg.setInputSource(source_cloud_);
  reg.setSearchMethodTarget(target_tree);
  reg.setSearchMethodSource(source_tree);
  reg.setTransformationEstimation(te_);
  reg.setCorrespondenceEstimation(ce_);
  for (auto &cj : cr_map) reg.addCorrespondenceRejector(cj.second);
  reg.setMaximumIterations(nr_iterations_);
  reg.setRANSACIterations(ransac_iterations_);
  reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
  reg.setMaxCorrespondenceDistance(distance_threshold_);
  reg.setTransformationEpsilon(transformation_epsilon_);
  reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
  reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

  reg.setResolution(resolution);
  reg.setStepSize(step_size);
  reg.setOulierRatio(outlier_ratio);

  reg.align(*ail_cloud);
  emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                          reg.getFinalTransformation().cast<float>(),
                          time.toc());
}

void Registration::TransformationEstimation2D() {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationEstimation2D<PointXYZRGBN, PointXYZRGBN> te;
  te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

void Registration::TransformationEstimation3Point() {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationEstimation3Point<PointXYZRGBN, PointXYZRGBN>
      te;
  te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

void Registration::TransformationEstimationDualQuaternion() {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationEstimationDualQuaternion<PointXYZRGBN,
                                                            PointXYZRGBN>
      te;
  te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

void Registration::TransformationEstimationLM() {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationEstimationLM<PointXYZRGBN, PointXYZRGBN> te;
  te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

void Registration::TransformationEstimationPointToPlane() {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationEstimationPointToPlane<PointXYZRGBN,
                                                          PointXYZRGBN>
      te;
  te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

void Registration::TransformationEstimationPointToPlaneLLS() {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationEstimationPointToPlaneLLS<PointXYZRGBN,
                                                             PointXYZRGBN>
      te;
  te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

void Registration::TransformationEstimationPointToPlaneLLSWeighted() {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<
      PointXYZRGBN, PointXYZRGBN>
      te;
  te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

void Registration::TransformationEstimationPointToPlaneWeighted() {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationEstimationPointToPlaneWeighted<PointXYZRGBN,
                                                                  PointXYZRGBN>
      te;
  te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

void Registration::TransformationEstimationSVD() {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationEstimationSVD<PointXYZRGBN, PointXYZRGBN> te;
  te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

void Registration::TransformationEstimationSymmetricPointToPlaneLLS(
    bool enforce_same_direction_normals) {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<
      PointXYZRGBN, PointXYZRGBN>
      te;
  te.setEnforceSameDirectionNormals(enforce_same_direction_normals);
  te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

void Registration::TransformationValidationEuclidean() {
  pcl::console::TicToc time;
  time.tic();
  Eigen::Matrix4f matrix;
  pcl::registration::TransformationValidationEuclidean<PointXYZRGBN,
                                                       PointXYZRGBN>
      te;
  te.validateTransformation(source_cloud_, target_cloud_, matrix);
  emit transformationEstimationResult(matrix, time.toc());
}

}  // namespace pca
