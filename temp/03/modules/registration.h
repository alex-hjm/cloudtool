#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include "common/cloud.h"
#include "modules/features.h"

class Registration
{

public:
    struct Result
    {
        int index=-1;
        float fitness_score=-1.0f;
        Eigen::Matrix4f final_transformation;
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };
    static Result SampleConsensusInitialAlignment(const Cloud::Ptr &targetCloud,const std::vector<Cloud::Ptr> &templateClouds,float normals_r,
                                                  float fpfh_r,int nr_iterations,double max_corrd_dis,int nr_samples,int k_correspondences,
                                                  float min_sample_distance);
    static Result SampleConsensusPrerejective(const Cloud::Ptr &targetCloud,const std::vector<Cloud::Ptr> &templateClouds,float normals_r,
                                              float fpfh_r,int nr_iterations,double max_corrd_dis,int nr_samples,int k_correspondences,
                                              float similarity_threshold,float inlier_fraction);

};






#endif // REGISTRATION_H
