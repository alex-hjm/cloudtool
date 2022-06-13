#include "registrations.h"

Registrations::Registrations()
{

}

CloudXYZRGBN::Ptr
Registrations::IterativeClosestPoint(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations, int ransac_iterations,
                                     double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, bool use_reciprocal_correspondence)
{
    time.tic ();
    pcl::IterativeClosestPoint<PointXYZRGBN, PointXYZRGBN> reg;
    CloudXYZRGBN::Ptr ail_cloud(new CloudXYZRGBN);
    if(te_type!=0)
        reg.setTransformationEstimation(this->getTransformationEstimation(te_type));
    if(ce_type>0&&ce_type<4)
        reg.setCorrespondenceEstimation(this->getCorrespondenceEstimation(ce_type));
    if(cr_type!=0)
        reg.addCorrespondenceRejector(this->getCorrespondenceRejector(cr_type));
    reg.setMaximumIterations(nr_iterations);
    reg.setRANSACIterations(ransac_iterations);
    reg.setRANSACOutlierRejectionThreshold(inlier_threshold);
    reg.setMaxCorrespondenceDistance(distance_threshold);
    reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    /////////////////////////////////////////////////
    reg.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    reg.setInputTarget(targetCloud);
    reg.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    reg.setInputSource(sourceCloud);
    reg.setSearchMethodSource(tree2);
    reg.align (*ail_cloud);
    tocTime=time.toc();
    if (reg.hasConverged ()) {
        score=reg.getFitnessScore();
        matrix=reg.getFinalTransformation().cast<float>();
    }
    else{
        score=-1;
        matrix=Eigen::Matrix4f::Identity ();
    }
    return ail_cloud;
}

CloudXYZRGBN::Ptr
Registrations::IterativeClosestPointWithNormals(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,
                                                double inlier_threshold,double distance_threshold, double euclidean_fitness_epsilon,
                                                bool use_reciprocal_correspondence,bool use_symmetric_objective,bool enforce_same_direction_normals)
{
    time.tic ();
    pcl::IterativeClosestPointWithNormals<PointXYZRGBN, PointXYZRGBN> regw;
    CloudXYZRGBN::Ptr ail_cloud(new CloudXYZRGBN);
    if(te_type!=0)
        regw.setTransformationEstimation(this->getTransformationEstimation(te_type));
    if(ce_type>0&&ce_type<4)
        regw.setCorrespondenceEstimation(this->getCorrespondenceEstimation(ce_type));
    if(cr_type!=0)
        regw.addCorrespondenceRejector(this->getCorrespondenceRejector(cr_type));
    regw.setMaximumIterations(nr_iterations);
    regw.setRANSACIterations(ransac_iterations);
    regw.setRANSACOutlierRejectionThreshold(inlier_threshold);
    regw.setMaxCorrespondenceDistance(distance_threshold);
    regw.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    /////////////////////////////////////////////////
    regw.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
    /////////////////////////////////////////////////
    regw.setUseSymmetricObjective(use_symmetric_objective);
    regw.setEnforceSameDirectionNormals(enforce_same_direction_normals);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    regw.setInputTarget(targetCloud);
    regw.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    regw.setInputSource(sourceCloud);
    regw.setSearchMethodSource(tree2);
    regw.align (*ail_cloud);
    tocTime=time.toc();
    if (regw.hasConverged ()){
        score=regw.getFitnessScore();
        matrix=regw.getFinalTransformation().cast<float>();
    }
    else{
        score=-1;
        matrix=Eigen::Matrix4f::Identity ();
    }
    return ail_cloud;

}

CloudXYZRGBN::Ptr
Registrations::IterativeClosestPointNonLinear(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,
                                              double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, bool use_reciprocal_correspondence)
{
    time.tic ();
    CloudXYZRGBN::Ptr ail_cloud(new CloudXYZRGBN);
    pcl::IterativeClosestPointNonLinear<PointXYZRGBN, PointXYZRGBN> regn;
    if(te_type!=0)
        regn.setTransformationEstimation(this->getTransformationEstimation(te_type));
    if(ce_type>0&&ce_type<4)
        regn.setCorrespondenceEstimation(this->getCorrespondenceEstimation(ce_type));
    if(cr_type!=0)
        regn.addCorrespondenceRejector(this->getCorrespondenceRejector(cr_type));
    regn.setMaximumIterations(nr_iterations);
    regn.setRANSACIterations(ransac_iterations);
    regn.setRANSACOutlierRejectionThreshold(inlier_threshold);
    regn.setMaxCorrespondenceDistance(distance_threshold);
    regn.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    /////////////////////////////////////////////////
    regn.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    regn.setInputTarget(targetCloud);
    regn.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    regn.setInputSource(sourceCloud);
    regn.setSearchMethodSource(tree2);
    regn.align (*ail_cloud);
    tocTime=time.toc();
    if (regn.hasConverged ()){
        score=regn.getFitnessScore();
        matrix=regn.getFinalTransformation().cast<float>();
    }
    else{
        score=-1;
        matrix=Eigen::Matrix4f::Identity ();
    }
    return ail_cloud;
}

CloudXYZRGBN::Ptr
Registrations::FPCSInitialAlignment(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,
                                    double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, float delta,float approx_overlap,
                                    float score_threshold,int nr_samples)
{
    time.tic ();
    pcl::registration::FPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN,PointXYZRGBN> fpcsi;
    CloudXYZRGBN::Ptr ail_cloud(new CloudXYZRGBN);
    if(te_type!=0)
        fpcsi.setTransformationEstimation(this->getTransformationEstimation(te_type));
    if(ce_type>0&&ce_type<4)
        fpcsi.setCorrespondenceEstimation(this->getCorrespondenceEstimation(ce_type));
    if(cr_type!=0)
        fpcsi.addCorrespondenceRejector(this->getCorrespondenceRejector(cr_type));
    fpcsi.setMaximumIterations(nr_iterations);
    fpcsi.setRANSACIterations(ransac_iterations);
    fpcsi.setRANSACOutlierRejectionThreshold(inlier_threshold);
    fpcsi.setMaxCorrespondenceDistance(distance_threshold);
    fpcsi.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    ////////////////////////////////////////////////
    fpcsi.setSourceNormals(sourceCloud );
    fpcsi.setTargetNormals(targetCloud);
    fpcsi.setDelta(delta);
    fpcsi.setApproxOverlap(approx_overlap);
    fpcsi.setScoreThreshold(score_threshold);
    fpcsi.setNumberOfSamples(nr_samples);
    ////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    fpcsi.setInputTarget(targetCloud);
    fpcsi.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    fpcsi.setInputSource(sourceCloud);
    fpcsi.setSearchMethodSource(tree2);
    fpcsi.setNumberOfThreads(12);
    fpcsi.align(*ail_cloud);
    tocTime=time.toc();
    if (fpcsi.hasConverged ()){
        score=fpcsi.getFitnessScore();
        matrix=fpcsi.getFinalTransformation().cast<float>();
    }
    else{
        score=-1;
        matrix=Eigen::Matrix4f::Identity ();
    }
    return ail_cloud;
}

CloudXYZRGBN::Ptr
Registrations::KFPCSInitialAlignment(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,
                                     double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, float delta,float approx_overlap,
                                     float score_threshold,int nr_samples,float upper_trl_boundary,float lower_trl_boundary,float lambda)
{
    time.tic ();
    pcl::registration::KFPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN,PointXYZRGBN> kfpcsi;
    CloudXYZRGBN::Ptr ail_cloud(new CloudXYZRGBN);
    if(te_type!=0)
        kfpcsi.setTransformationEstimation(this->getTransformationEstimation(te_type));
    if(ce_type>0&&ce_type<4)
        kfpcsi.setCorrespondenceEstimation(this->getCorrespondenceEstimation(ce_type));
    if(cr_type!=0)
        kfpcsi.addCorrespondenceRejector(this->getCorrespondenceRejector(cr_type));
    kfpcsi.setMaximumIterations(nr_iterations);
    kfpcsi.setRANSACIterations(ransac_iterations);
    kfpcsi.setRANSACOutlierRejectionThreshold(inlier_threshold);
    kfpcsi.setMaxCorrespondenceDistance(distance_threshold);
    kfpcsi.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    ////////////////////////////////////////////////
    kfpcsi.setSourceNormals(sourceCloud);
    kfpcsi.setTargetNormals(targetCloud);
    kfpcsi.setDelta(delta);
    kfpcsi.setApproxOverlap(approx_overlap);
    kfpcsi.setScoreThreshold(score_threshold);
    kfpcsi.setNumberOfSamples(nr_samples);
    /////////////////////////////////////////////////
    kfpcsi.setUpperTranslationThreshold(upper_trl_boundary);
    kfpcsi.setLowerTranslationThreshold(lower_trl_boundary);
    kfpcsi.setLambda(lambda);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    kfpcsi.setInputTarget(targetCloud);
    kfpcsi.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    kfpcsi.setInputSource(sourceCloud);
    kfpcsi.setSearchMethodSource(tree2);
    kfpcsi.setNumberOfThreads(12);
    kfpcsi.align(*ail_cloud);
    tocTime=time.toc();
    if (kfpcsi.hasConverged ()){
        score=kfpcsi.getFitnessScore();
        matrix=kfpcsi.getFinalTransformation().cast<float>();
    }
    else{
        score=-1;
        matrix=Eigen::Matrix4f::Identity ();
    }
    return ail_cloud;
}

CloudXYZRGBN::Ptr
Registrations::SampleConsensusPrerejective(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const FPFHFeature::Ptr &targetFeature,
                                           const FPFHFeature::Ptr &sourceFeature,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,
                                           double euclidean_fitness_epsilon,int nr_samples, int k, float similarity_threshold,float inlier_fraction)
{
    time.tic ();
    pcl::SampleConsensusPrerejective<PointXYZRGBN, PointXYZRGBN,pcl::FPFHSignature33> scp;
    CloudXYZRGBN::Ptr ail_cloud(new CloudXYZRGBN);
    if(te_type!=0)
        scp.setTransformationEstimation(this->getTransformationEstimation(te_type));
    if(ce_type>0&&ce_type<4)
        scp.setCorrespondenceEstimation(this->getCorrespondenceEstimation(ce_type));
    if(cr_type!=0)
        scp.addCorrespondenceRejector(this->getCorrespondenceRejector(cr_type));
    scp.setMaximumIterations(nr_iterations);
    scp.setRANSACIterations(ransac_iterations);
    scp.setRANSACOutlierRejectionThreshold(inlier_threshold);
    scp.setMaxCorrespondenceDistance(distance_threshold);
    scp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    ////////////////////////////////////////////////
    scp.setTargetFeatures(targetFeature);
    scp.setSourceFeatures(sourceFeature);
    scp.setNumberOfSamples(nr_samples);
    scp.setCorrespondenceRandomness(k);
    scp.setSimilarityThreshold(similarity_threshold);
    scp.setInlierFraction(inlier_fraction);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    scp.setInputTarget(targetCloud);
    scp.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    scp.setInputSource(sourceCloud);
    scp.setSearchMethodSource(tree2);
    scp.align (*ail_cloud);
    tocTime=time.toc();
    if (scp.hasConverged ()){
        score=scp.getFitnessScore();
        matrix=scp.getFinalTransformation().cast<float>();
    }
    else{
        score=-1;
        matrix=Eigen::Matrix4f::Identity ();
    }
    return ail_cloud;
}

CloudXYZRGBN::Ptr
Registrations::SampleConsensusPrerejective(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const SHOTFeature::Ptr &targetFeature,
                                           const SHOTFeature::Ptr &sourceFeature,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,
                                           double euclidean_fitness_epsilon,int nr_samples, int k, float similarity_threshold,float inlier_fraction)
{
    time.tic ();
    pcl::SampleConsensusPrerejective<PointXYZRGBN, PointXYZRGBN,pcl::SHOT352> scp;
    CloudXYZRGBN::Ptr ail_cloud(new CloudXYZRGBN);
    if(te_type!=0)
        scp.setTransformationEstimation(this->getTransformationEstimation(te_type));
    if(ce_type>0&&ce_type<4)
        scp.setCorrespondenceEstimation(this->getCorrespondenceEstimation(ce_type));
    if(cr_type!=0)
        scp.addCorrespondenceRejector(this->getCorrespondenceRejector(cr_type));
    scp.setMaximumIterations(nr_iterations);
    scp.setRANSACIterations(ransac_iterations);
    scp.setRANSACOutlierRejectionThreshold(inlier_threshold);
    scp.setMaxCorrespondenceDistance(distance_threshold);
    scp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    ////////////////////////////////////////////////
    scp.setTargetFeatures(targetFeature);
    scp.setSourceFeatures(sourceFeature);
    scp.setNumberOfSamples(nr_samples);
    scp.setCorrespondenceRandomness(k);
    scp.setSimilarityThreshold(similarity_threshold);
    scp.setInlierFraction(inlier_fraction);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    scp.setInputTarget(targetCloud);
    scp.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    scp.setInputSource(sourceCloud);
    scp.setSearchMethodSource(tree2);
    scp.align (*ail_cloud);
    tocTime=time.toc();
    if (scp.hasConverged ()){
        score=scp.getFitnessScore();
        matrix=scp.getFinalTransformation().cast<float>();
    }
    else{
        score=-1;
        matrix=Eigen::Matrix4f::Identity ();
    }
    return ail_cloud;
}

CloudXYZRGBN::Ptr
Registrations::SampleConsensusInitialAlignment(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const FPFHFeature::Ptr &targetFeature,
                                               const FPFHFeature::Ptr &sourceFeature,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,
                                               double euclidean_fitness_epsilon,int nr_samples, int k,float min_sample_distance)
{
    time.tic ();
    pcl::SampleConsensusInitialAlignment<PointXYZRGBN, PointXYZRGBN,pcl::FPFHSignature33> scia;
    CloudXYZRGBN::Ptr ail_cloud(new CloudXYZRGBN);
    if(te_type!=0)
        scia.setTransformationEstimation(this->getTransformationEstimation(te_type));
    if(ce_type>0&&ce_type<4)
        scia.setCorrespondenceEstimation(this->getCorrespondenceEstimation(ce_type));
    if(cr_type!=0)
        scia.addCorrespondenceRejector(this->getCorrespondenceRejector(cr_type));
    scia.setMaximumIterations(nr_iterations);
    scia.setRANSACIterations(ransac_iterations);
    scia.setRANSACOutlierRejectionThreshold(inlier_threshold);
    scia.setMaxCorrespondenceDistance(distance_threshold);
    scia.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    ////////////////////////////////////////////////
    scia.setTargetFeatures(targetFeature);
    scia.setSourceFeatures(sourceFeature);
    scia.setNumberOfSamples(nr_samples);
    scia.setCorrespondenceRandomness(k);
    scia.setMinSampleDistance(min_sample_distance);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    scia.setInputTarget(targetCloud);
    scia.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    scia.setInputSource(sourceCloud);
    scia.setSearchMethodSource(tree2);
    scia.align (*ail_cloud);
    tocTime=time.toc();
    if (scia.hasConverged ()){
        score=scia.getFitnessScore();
        matrix=scia.getFinalTransformation().cast<float>();
    }
    else{
        score=-1;
        matrix=Eigen::Matrix4f::Identity ();
    }
    return ail_cloud;
}

CloudXYZRGBN::Ptr
Registrations::SampleConsensusInitialAlignment(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,const SHOTFeature::Ptr &targetFeature,
                                               const SHOTFeature::Ptr &sourceFeature,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,
                                               double euclidean_fitness_epsilon,int nr_samples, int k,float min_sample_distance)
{
    time.tic ();
    pcl::SampleConsensusInitialAlignment<PointXYZRGBN, PointXYZRGBN,pcl::SHOT352> scia;
    CloudXYZRGBN::Ptr ail_cloud(new CloudXYZRGBN);
    if(te_type!=0)
        scia.setTransformationEstimation(this->getTransformationEstimation(te_type));
    if(ce_type>0&&ce_type<4)
        scia.setCorrespondenceEstimation(this->getCorrespondenceEstimation(ce_type));
    if(cr_type!=0)
        scia.addCorrespondenceRejector(this->getCorrespondenceRejector(cr_type));
    scia.setMaximumIterations(nr_iterations);
    scia.setRANSACIterations(ransac_iterations);
    scia.setRANSACOutlierRejectionThreshold(inlier_threshold);
    scia.setMaxCorrespondenceDistance(distance_threshold);
    scia.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    ////////////////////////////////////////////////
    scia.setTargetFeatures(targetFeature);
    scia.setSourceFeatures(sourceFeature);
    scia.setNumberOfSamples(nr_samples);
    scia.setCorrespondenceRandomness(k);
    scia.setMinSampleDistance(min_sample_distance);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    scia.setInputTarget(targetCloud);
    scia.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    scia.setInputSource(sourceCloud);
    scia.setSearchMethodSource(tree2);
    scia.align (*ail_cloud);
    tocTime=time.toc();
    if (scia.hasConverged ()){
        score=scia.getFitnessScore();
        matrix=scia.getFinalTransformation().cast<float>();
    }
    else{
        score=-1;
        matrix=Eigen::Matrix4f::Identity ();
    }
    return ail_cloud;
}

CloudXYZRGBN::Ptr
Registrations::NormalDistributionsTransform(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,int te_type,int ce_type,int cr_type,int nr_iterations,int ransac_iterations,
                                            double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, float resolution,double step_size,double outlier_ratio)
{
    time.tic ();
    pcl::NormalDistributionsTransform<PointXYZRGBN, PointXYZRGBN> ndt;
    CloudXYZRGBN::Ptr ail_cloud(new CloudXYZRGBN);
    if(te_type!=0)
        ndt.setTransformationEstimation(this->getTransformationEstimation(te_type));
    if(ce_type>0&&ce_type<4)
        ndt.setCorrespondenceEstimation(this->getCorrespondenceEstimation(ce_type));
    if(cr_type!=0)
        ndt.addCorrespondenceRejector(this->getCorrespondenceRejector(cr_type));
    ndt.setMaximumIterations(nr_iterations);
    ndt.setRANSACIterations(ransac_iterations);
    ndt.setRANSACOutlierRejectionThreshold(inlier_threshold);
    ndt.setMaxCorrespondenceDistance(distance_threshold);
    ndt.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    /////////////////////////////////////////////////
    ndt.setResolution(resolution);
    ndt.setStepSize(step_size);
    ndt.setOulierRatio(outlier_ratio);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    ndt.setInputTarget(targetCloud);
    ndt.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    ndt.setInputSource(sourceCloud);
    ndt.setSearchMethodSource(tree2);
    ndt.align (*ail_cloud);
    tocTime=time.toc();
    if (ndt.hasConverged ()){
        score=ndt.getFitnessScore();
        matrix=ndt.getFinalTransformation().cast<float>();
    }
    else{
        score=-1;
        matrix=Eigen::Matrix4f::Identity ();
    }
    return ail_cloud;
}

CorrespondencesPtr
Registrations::CorrespondenceEstimation(const CloudXYZRGBN::Ptr &targetCloud, const CloudXYZRGBN::Ptr &sourceCloud)
{
    CorrespondencesPtr all_correspondence(new Correspondences);
    ce.reset(new pcl::registration::CorrespondenceEstimation<PointXYZRGBN,PointXYZRGBN>);
    ce->setInputTarget(targetCloud);
    ce->setInputSource(sourceCloud);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    ce->setSearchMethodSource(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    ce->setSearchMethodTarget(tree2);
    ce->determineReciprocalCorrespondences(*all_correspondence);
    return all_correspondence;
}

CorrespondencesPtr
Registrations::CorrespondenceEstimationBackprojection(const CloudXYZRGBN::Ptr &targetCloud, const CloudXYZRGBN::Ptr &sourceCloud, int k)
{
    CorrespondencesPtr all_correspondence(new Correspondences);
    cebp.reset(new pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBN,PointXYZRGBN,PointXYZRGBN>);
    cebp->setInputTarget(targetCloud);
    cebp->setInputSource(sourceCloud);
    cebp->setSourceNormals(sourceCloud);
    cebp->setTargetNormals(targetCloud);
    cebp->setKSearch(k);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    cebp->setSearchMethodSource(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    cebp->setSearchMethodTarget(tree2);
    cebp->determineReciprocalCorrespondences(*all_correspondence);
    return  all_correspondence;
}

CorrespondencesPtr
Registrations::CorrespondenceEstimationNormalShooting(const CloudXYZRGBN::Ptr &targetCloud, const CloudXYZRGBN::Ptr &sourceCloud, int k)
{
    CorrespondencesPtr all_correspondence(new Correspondences);
    cens.reset(new pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBN,PointXYZRGBN,PointXYZRGBN>);;
    cens->setInputTarget(targetCloud);
    cens->setInputSource(sourceCloud);
    cens->setSourceNormals(sourceCloud);
    cens->setKSearch(k);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    cens->setSearchMethodSource(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    cens->setSearchMethodTarget(tree2);
    cens->determineReciprocalCorrespondences(*all_correspondence);
    return  all_correspondence;
}

CorrespondencesPtr
Registrations::CorrespondenceEstimation(const FPFHFeature::Ptr &targetFeature,const FPFHFeature::Ptr &sourceFeature)
{
    CorrespondencesPtr all_correspondence(new Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,pcl::FPFHSignature33> ce;
    ce.setInputTarget(targetFeature);
    ce.setInputSource(sourceFeature);
    pcl::search::KdTree<pcl::FPFHSignature33>::Ptr tree1(new pcl::search::KdTree<pcl::FPFHSignature33>);
    ce.setSearchMethodSource(tree1);
    pcl::search::KdTree<pcl::FPFHSignature33>::Ptr tree2(new pcl::search::KdTree<pcl::FPFHSignature33>);
    ce.setSearchMethodTarget(tree2);
    ce.determineReciprocalCorrespondences(*all_correspondence);
    return  all_correspondence;
}

CorrespondencesPtr
Registrations::CorrespondenceEstimation(const SHOTFeature::Ptr &targetFeature,const SHOTFeature::Ptr &sourceFeature)
{
    CorrespondencesPtr all_correspondence(new Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::SHOT352,pcl::SHOT352> ce;
    ce.setInputTarget(targetFeature);
    ce.setInputSource(sourceFeature);
    pcl::search::KdTree<pcl::SHOT352>::Ptr tree1(new pcl::search::KdTree<pcl::SHOT352>);
    ce.setSearchMethodSource(tree1);
    pcl::search::KdTree<pcl::SHOT352>::Ptr tree2(new pcl::search::KdTree<pcl::SHOT352>);
    ce.setSearchMethodTarget(tree2);
    ce.determineReciprocalCorrespondences(*all_correspondence);
    return  all_correspondence;
}

CorrespondencesPtr
Registrations::CorrespondenceEstimationFPFHKDTree(const FPFHFeature::Ptr &targetFeature,const FPFHFeature::Ptr &sourceFeature, double threshold)
{
    CorrespondencesPtr all_correspondence(new Correspondences);
    pcl::search::KdTree<pcl::FPFHSignature33> match_search;
    match_search.setInputCloud(sourceFeature);
    for (size_t i = 0; i < targetFeature->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        int found_neighs = match_search.nearestKSearch (targetFeature->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < threshold)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            all_correspondence->push_back (corr);
        }
    }
    return all_correspondence;
}

CorrespondencesPtr
Registrations::CorrespondenceEstimationSHOTKDTree(const SHOTFeature::Ptr &targetFeature, const SHOTFeature::Ptr &sourceFeature, double threshold)
{
    CorrespondencesPtr all_correspondence(new Correspondences);
    pcl::search::KdTree<pcl::SHOT352> match_search;
    match_search.setInputCloud(sourceFeature);
    for (size_t i = 0; i < targetFeature->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!pcl_isfinite (targetFeature->at(i).descriptor[0]))
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch (targetFeature->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < threshold)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            all_correspondence->push_back (corr);
        }
    }
    return all_correspondence;
}

TransformationEstimationPtr
Registrations::TransformationEstimationLM()
{
    pcl::registration::TransformationEstimationLM<PointXYZRGBN,PointXYZRGBN>::Ptr telm
            (new pcl::registration::TransformationEstimationLM<PointXYZRGBN,PointXYZRGBN>);
    return telm;
}

TransformationEstimationPtr
Registrations::TransformationEstimationPointToPlane()
{
    pcl::registration::TransformationEstimationPointToPlane<PointXYZRGBN,PointXYZRGBN>::Ptr teptp
            (new pcl::registration::TransformationEstimationPointToPlane<PointXYZRGBN,PointXYZRGBN>);
    return teptp;
}

TransformationEstimationPtr
Registrations::TransformationEstimationPointToPlaneLLS()
{
    pcl::registration::TransformationEstimationPointToPlaneLLS<PointXYZRGBN,PointXYZRGBN>::Ptr teptplls
            (new pcl::registration::TransformationEstimationPointToPlaneLLS<PointXYZRGBN,PointXYZRGBN>);
    return teptplls;
}

TransformationEstimationPtr
Registrations::TransformationEstimationPointToPlaneLLSWeighted()
{
    pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<PointXYZRGBN,PointXYZRGBN>::Ptr teptpllsw
            (new pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<PointXYZRGBN,PointXYZRGBN>);
    return teptpllsw;
}

TransformationEstimationPtr
Registrations::TransformationEstimationSVD()
{
    pcl::registration::TransformationEstimationSVD<PointXYZRGBN,PointXYZRGBN>::Ptr tesvd
            (new pcl::registration::TransformationEstimationSVD<PointXYZRGBN,PointXYZRGBN>);
    return tesvd;
}

TransformationEstimationPtr
Registrations::TransformationEstimationSVDScale()
{
    pcl::registration::TransformationEstimationSVDScale<PointXYZRGBN,PointXYZRGBN>::Ptr te
            (new pcl::registration::TransformationEstimationSVDScale<PointXYZRGBN,PointXYZRGBN>);
    return te;
}

TransformationEstimationPtr
Registrations::TransformationEstimationSymmetricPointToPlaneLLS()
{
    pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointXYZRGBN,PointXYZRGBN>::Ptr tesptplls
            (new pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointXYZRGBN,PointXYZRGBN>);
    return tesptplls;
}

CorrespondencesPtr
Registrations::CorrespondenceRejectorDistance(const CloudXYZRGBN::Ptr &targetCloud,const CloudXYZRGBN::Ptr &sourceCloud,
                                              const CorrespondencesPtr &original_correspondences,float max_distance)
{
    CorrespondencesPtr remaining_correspondences(new Correspondences);
    crd.reset(new pcl::registration::CorrespondenceRejectorDistance);
    crd->setInputTarget<PointXYZRGBN>(targetCloud);
    crd->setInputSource<PointXYZRGBN>(sourceCloud);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    crd->setSearchMethodTarget<PointXYZRGBN>(tree);
    crd->setMaximumDistance(max_distance);
    crd->setInputCorrespondences(original_correspondences);
    crd->getCorrespondences(*remaining_correspondences);
    return remaining_correspondences;
}

CorrespondencesPtr
Registrations::CorrespondenceRejectorMedianDistance(const CloudXYZRGBN::Ptr &targetCloud, const CloudXYZRGBN::Ptr &sourceCloud,
                                                    const CorrespondencesPtr &original_correspondences, double factor)
{
    CorrespondencesPtr remaining_correspondences(new Correspondences);
    crmd.reset(new pcl::registration::CorrespondenceRejectorMedianDistance);
    crmd->setInputTarget<PointXYZRGBN>(targetCloud);
    crmd->setInputSource<PointXYZRGBN>(sourceCloud);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    crmd->setSearchMethodTarget<PointXYZRGBN>(tree);
    crmd->setMedianFactor(factor);
    crmd->setInputCorrespondences(original_correspondences);
    crmd->getCorrespondences(*remaining_correspondences);
    return remaining_correspondences;
}

CorrespondencesPtr
Registrations::CorrespondenceRejectorOneToOne(const CorrespondencesPtr &original_correspondences)
{
    CorrespondencesPtr remaining_correspondences(new Correspondences);
    croto.reset(new pcl::registration::CorrespondenceRejectorOneToOne);
    croto->setInputCorrespondences(original_correspondences);
    croto->getCorrespondences(*remaining_correspondences);
    return remaining_correspondences;
}

CorrespondencesPtr
Registrations::CorrespondenceRejectionOrganizedBoundary(const CorrespondencesPtr &original_correspondences)
{
    CorrespondencesPtr remaining_correspondences(new Correspondences);
    crob.reset(new pcl::registration::CorrespondenceRejectionOrganizedBoundary);
    crob->setInputCorrespondences(original_correspondences);
    crob->getCorrespondences(*remaining_correspondences);
    return remaining_correspondences;
}

CorrespondencesPtr
Registrations::CorrespondenceRejectorPoly(const CloudXYZRGBN::Ptr &targetCloud, const CloudXYZRGBN::Ptr &sourceCloud,const CorrespondencesPtr &original_correspondences,
                                          int cardinality, float similarity_threshold,int iterations)
{
    CorrespondencesPtr remaining_correspondences(new Correspondences);
    crp.reset(new pcl::registration::CorrespondenceRejectorPoly<PointXYZRGBN,PointXYZRGBN> );
    crp->setInputTarget(targetCloud);
    crp->setInputSource(sourceCloud);
    crp->setCardinality(cardinality);
    crp->setSimilarityThreshold(similarity_threshold);
    crp->setIterations(iterations);
    crp->setInputCorrespondences(original_correspondences);
    crp->getCorrespondences(*remaining_correspondences);
    return remaining_correspondences;
}

CorrespondencesPtr
Registrations::CorrespondenceRejectorSampleConsensus(const CloudXYZRGBN::Ptr &targetCloud, const CloudXYZRGBN::Ptr &sourceCloud, const CorrespondencesPtr &original_correspondences,
                                                     double threshold, int max_iterations, const bool refine, bool s)
{
    CorrespondencesPtr remaining_correspondences(new Correspondences);
    crsc.reset(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBN> );
    crsc->setInputTarget(targetCloud);
    crsc->setInputSource(sourceCloud);
    crsc->setInlierThreshold(threshold);
    crsc->setMaximumIterations(max_iterations);
    crsc->setRefineModel(refine);
    crsc->setSaveInliers(s);
    crsc->setInputCorrespondences(original_correspondences);
    crsc->getCorrespondences(*remaining_correspondences);
    return remaining_correspondences;
}

CorrespondencesPtr
Registrations::CorrespondenceRejectorSurfaceNormal(const CloudXYZRGBN::Ptr &targetCloud, const CloudXYZRGBN::Ptr &sourceCloud, const CorrespondencesPtr &original_correspondences,
                                                   double threshold)
{
    CorrespondencesPtr remaining_correspondences(new Correspondences);
    crsn.reset(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
    crsn->setInputTarget<PointXYZRGBN> (targetCloud);
    crsn->setInputSource<PointXYZRGBN> (sourceCloud);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    crsn->setSearchMethodTarget<PointXYZRGBN>(tree);
    crsn->setThreshold(threshold);
    crsn->setInputCorrespondences(original_correspondences);
    crsn->getCorrespondences(*remaining_correspondences);
    return remaining_correspondences;
}

CorrespondencesPtr
Registrations::CorrespondenceRejectorTrimmed(const CorrespondencesPtr &original_correspondences,float ratio, int min_correspondences)
{
    CorrespondencesPtr remaining_correspondences(new Correspondences);
    crt.reset(new pcl::registration::CorrespondenceRejectorTrimmed);
    crt->setOverlapRatio(ratio);
    crt->setMinCorrespondences(min_correspondences);
    crt->setInputCorrespondences(original_correspondences);
    crt->getCorrespondences(*remaining_correspondences);
    return remaining_correspondences;
}

CorrespondencesPtr
Registrations::CorrespondenceRejectorVarTrimmed(const CloudXYZRGBN::Ptr &targetCloud, const CloudXYZRGBN::Ptr &sourceCloud, const CorrespondencesPtr &original_correspondences,
                                               float min_ratio, double max_ratio)
{
    CorrespondencesPtr remaining_correspondences(new Correspondences);
    crvt.reset(new pcl::registration::CorrespondenceRejectorVarTrimmed);
    crvt->setInputTarget<PointXYZRGBN>(targetCloud);
    crvt->setInputSource<PointXYZRGBN>(sourceCloud);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    crvt->setSearchMethodTarget<PointXYZRGBN>(tree);
    crvt->setMinRatio(min_ratio);
    crvt->setMaxRatio(max_ratio);
    crvt->setInputCorrespondences(original_correspondences);
    crvt->getCorrespondences(*remaining_correspondences);
    return remaining_correspondences;
}

CorrespondenceEstimationPtr
Registrations::getCorrespondenceEstimation(int ce_type)
{
    switch (ce_type) {
    case 1:
        return ce;
    case 2:
        return cebp;
    case 3:
        return cens;
    }
}

TransformationEstimationPtr
Registrations::getTransformationEstimation(int te_type)
{
    switch (te_type) {
    case 1:
        return this->TransformationEstimationLM();
    case 2:
        return this->TransformationEstimationPointToPlane();
    case 3:
        return this->TransformationEstimationPointToPlaneLLS();
    case 4:
        return this->TransformationEstimationPointToPlaneLLSWeighted();
    case 5:
        return this->TransformationEstimationSVD();
    case 6:
        return this->TransformationEstimationSVDScale();
    case 7:
        return this->TransformationEstimationSymmetricPointToPlaneLLS();
    }
}

CorrespondenceRejectorPtr
Registrations::getCorrespondenceRejector(int cr_type)
{
    switch (cr_type) {
    case 1:
        return crd;
    case 2:
        return crmd;
    case 3:
        return croto;
    case 4:
        return crp;
    case 5:
        return crsc;
    case 6:
        return crsn;
    case 7:
        return crt;
    case 8:
        return crvt;
    }
}





