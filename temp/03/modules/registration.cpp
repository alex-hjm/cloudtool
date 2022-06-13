#include "registration.h"



Registration::Result Registration::SampleConsensusInitialAlignment(const Cloud::Ptr &targetCloud,const std::vector<Cloud::Ptr> &templateClouds,float normals_r,
                                                                   float fpfh_r,int nr_iterations,double max_corrd_dis,int nr_samples,int k_correspondences,
                                                                   float min_sample_distance)
{
    for(auto &i:templateClouds) {
        i->normals=Features::NormalEstimation(i,normals_r);
        i->fpfh_feature=Features::FPFHEstimation(i,fpfh_r);
    }
    targetCloud->normals=Features::NormalEstimation(targetCloud,normals_r);
    targetCloud->fpfh_feature=Features::FPFHEstimation(targetCloud,fpfh_r);
    pcl::SampleConsensusInitialAlignment<PointXYZRGBN, PointXYZRGBN, pcl::FPFHSignature33> sac_ia;
    sac_ia.setMaximumIterations(nr_iterations);
    sac_ia.setMaxCorrespondenceDistance(max_corrd_dis);
    sac_ia.setMinSampleDistance(min_sample_distance);
    sac_ia.setNumberOfSamples(nr_samples);
    sac_ia.setCorrespondenceRandomness(k_correspondences);
    sac_ia.setInputTarget(targetCloud);
    sac_ia.setTargetFeatures(targetCloud->fpfh_feature);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    sac_ia.setSearchMethodTarget(tree1);
    sac_ia.setSearchMethodSource(tree2);

    Result best_result;
    float lowest_score = std::numeric_limits<float>::infinity ();
    for(std::size_t i = 0; i < templateClouds.size (); ++i) {
        Cloud ail_cloud;
        sac_ia.setInputSource(templateClouds[i]);
        sac_ia.setSourceFeatures(templateClouds[i]->fpfh_feature);
        sac_ia.align (ail_cloud);
        if (sac_ia.hasConverged ()) {
            if((float) sac_ia.getFitnessScore (max_corrd_dis)<lowest_score) {
                lowest_score=(float) sac_ia.getFitnessScore (max_corrd_dis);
                best_result.fitness_score=lowest_score;
                best_result.final_transformation=sac_ia.getFinalTransformation ();
                best_result.index=i;
            }
        }
    }
    return best_result;
}

Registration::Result Registration::SampleConsensusPrerejective(const Cloud::Ptr &targetCloud,const std::vector<Cloud::Ptr> &templateClouds,float normals_r,
                                                               float fpfh_r,int nr_iterations,double max_corrd_dis,int nr_samples,int k_correspondences,
                                                               float similarity_threshold,float inlier_fraction)
{
    for(auto &i:templateClouds) {
        i->normals=Features::NormalEstimation(i,normals_r);
        i->fpfh_feature=Features::FPFHEstimation(i,fpfh_r);
    }
    targetCloud->normals=Features::NormalEstimation(targetCloud,normals_r);
    targetCloud->fpfh_feature=Features::FPFHEstimation(targetCloud,fpfh_r);
    pcl::SampleConsensusPrerejective<PointXYZRGBN, PointXYZRGBN, pcl::FPFHSignature33> sac_pj;
    sac_pj.setMaximumIterations(nr_iterations);
    sac_pj.setMaxCorrespondenceDistance(max_corrd_dis);
    sac_pj.setSimilarityThreshold(similarity_threshold);
    sac_pj.setNumberOfSamples(nr_samples);
    sac_pj.setCorrespondenceRandomness(k_correspondences);
    sac_pj.setInlierFraction(inlier_fraction);
    sac_pj.setInputTarget(targetCloud);
    sac_pj.setTargetFeatures(targetCloud->fpfh_feature);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    sac_pj.setSearchMethodTarget(tree1);
    sac_pj.setSearchMethodSource(tree2);

    Result best_result;
    float lowest_score = std::numeric_limits<float>::infinity ();
    for(std::size_t i = 0; i < templateClouds.size (); ++i) {
        Cloud ail_cloud;
        sac_pj.setInputSource(templateClouds[i]);
        sac_pj.setSourceFeatures(templateClouds[i]->fpfh_feature);
        sac_pj.align (ail_cloud);
        if (sac_pj.hasConverged ()) {
            if((float) sac_pj.getFitnessScore (max_corrd_dis)<lowest_score) {
                lowest_score=(float) sac_pj.getFitnessScore (max_corrd_dis);
                best_result.fitness_score=lowest_score;
                best_result.final_transformation=sac_pj.getFinalTransformation ();
                best_result.index=i;
            }
        }
    }
    return best_result;
}
