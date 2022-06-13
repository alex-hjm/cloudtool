#include "registration.h"

Registration::Registration(QObject *parent) : QObject(parent)
{

}

void
Registration::IterativeClosestPoint(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations, int ransac_iterations,
                                    double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon, bool use_reciprocal_correspondence)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::IterativeClosestPoint<PointXYZRGBN, PointXYZRGBN> reg;
    Cloud::Ptr ail_cloud(new Cloud);ail_cloud->copyInfo(sourceCloud);
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
    if (reg.hasConverged ())
        emit result(true,ail_cloud,reg.getFinalTransformation().cast<float>(),reg.getFitnessScore(),time.toc());
    else
        emit result(false,ail_cloud,Eigen::Matrix4f::Identity(),-1,time.toc());
}

void
Registration::IterativeClosestPointWithNormals(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                               double distance_threshold, double euclidean_fitness_epsilon,bool use_reciprocal_correspondence,bool use_symmetric_objective,
                                               bool enforce_same_direction_normals)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::IterativeClosestPointWithNormals<PointXYZRGBN, PointXYZRGBN> regw;
    Cloud::Ptr ail_cloud(new Cloud);ail_cloud->copyInfo(sourceCloud);
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
    if (regw.hasConverged ())
        emit result(true,ail_cloud,regw.getFinalTransformation().cast<float>(),regw.getFitnessScore(),time.toc());
    else
        emit result(false,ail_cloud,Eigen::Matrix4f::Identity(),-1,time.toc());

}

void
Registration::IterativeClosestPointNonLinear(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                             double distance_threshold,double euclidean_fitness_epsilon, bool use_reciprocal_correspondence)
{
    pcl::console::TicToc time;
    time.tic ();
    Cloud::Ptr ail_cloud(new Cloud);ail_cloud->copyInfo(sourceCloud);
    pcl::IterativeClosestPointNonLinear<PointXYZRGBN, PointXYZRGBN> regn;
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
    if (regn.hasConverged ())
        emit result(true,ail_cloud,regn.getFinalTransformation().cast<float>(),regn.getFitnessScore(),time.toc());
    else
        emit result(false,ail_cloud,Eigen::Matrix4f::Identity(),-1,time.toc());
}

void
Registration::NormalDistributionsTransform(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                           double distance_threshold,double euclidean_fitness_epsilon, float resolution,double step_size,double outlier_ratio)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::NormalDistributionsTransform<PointXYZRGBN, PointXYZRGBN> ndt;
    Cloud::Ptr ail_cloud(new Cloud);ail_cloud->copyInfo(sourceCloud);
    ndt.setMaximumIterations(nr_iterations);
    ndt.setRANSACIterations(ransac_iterations);
    ndt.setRANSACOutlierRejectionThreshold(inlier_threshold);
    ndt.setMaxCorrespondenceDistance(distance_threshold);
    ndt.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    /////////////////////////////////////////////////
    ndt.setResolution(resolution);
    ndt.setStepSize(step_size);
    //ndt.setOulierRatio(outlier_ratio);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    ndt.setInputTarget(targetCloud);
    ndt.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    ndt.setInputSource(sourceCloud);
    ndt.setSearchMethodSource(tree2);
    ndt.align (*ail_cloud);
    if (ndt.hasConverged ())
        emit result(true,ail_cloud,ndt.getFinalTransformation().cast<float>(),ndt.getFitnessScore(),time.toc());
    else
        emit result(false,ail_cloud,Eigen::Matrix4f::Identity(),-1,time.toc());
}

void
Registration::SampleConsensusPrerejectiveFPFH(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,const FPFHFeature::Ptr &targetFeature,const FPFHFeature::Ptr &sourceFeature,
                                          int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                          int nr_samples, int k, float similarity_threshold,float inlier_fraction)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::SampleConsensusPrerejective<PointXYZRGBN, PointXYZRGBN,pcl::FPFHSignature33> scp;
    Cloud::Ptr ail_cloud(new Cloud);ail_cloud->copyInfo(sourceCloud);
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
    if (scp.hasConverged ())
        emit result(true,ail_cloud,scp.getFinalTransformation().cast<float>(),scp.getFitnessScore(),time.toc());
    else
        emit result(false,ail_cloud,Eigen::Matrix4f::Identity(),-1,time.toc());
}

void
Registration::SampleConsensusPrerejectiveSHOT(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,const SHOTFeature::Ptr &targetFeature,const SHOTFeature::Ptr &sourceFeature,
                                          int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                          int nr_samples, int k, float similarity_threshold,float inlier_fraction)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::SampleConsensusPrerejective<PointXYZRGBN, PointXYZRGBN,pcl::SHOT352> scp;
    Cloud::Ptr ail_cloud(new Cloud);ail_cloud->copyInfo(sourceCloud);
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
    if (scp.hasConverged ())
        emit result(true,ail_cloud,scp.getFinalTransformation().cast<float>(),scp.getFitnessScore(),time.toc());
    else
        emit result(false,ail_cloud,Eigen::Matrix4f::Identity(),-1,time.toc());
}

void
Registration::SampleConsensusInitialAlignmentFPFH(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,const FPFHFeature::Ptr &targetFeature,const FPFHFeature::Ptr &sourceFeature,
                                              int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                              int nr_samples, int k,float min_sample_distance)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::SampleConsensusInitialAlignment<PointXYZRGBN, PointXYZRGBN,pcl::FPFHSignature33> scia;
    Cloud::Ptr ail_cloud(new Cloud);ail_cloud->copyInfo(sourceCloud);
    scia.setMaximumIterations(nr_iterations);
//    scia.setRANSACIterations(ransac_iterations);
//    scia.setRANSACOutlierRejectionThreshold(inlier_threshold);
    scia.setMaxCorrespondenceDistance(distance_threshold);
//    scia.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    ////////////////////////////////////////////////
    scia.setTargetFeatures(targetFeature);
    scia.setSourceFeatures(sourceFeature);
//    scia.setNumberOfSamples(nr_samples);
//    scia.setCorrespondenceRandomness(k);
    scia.setMinSampleDistance(min_sample_distance);
    /////////////////////////////////////////////////
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    scia.setInputTarget(targetCloud);
    scia.setSearchMethodTarget(tree1);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBN>);
    scia.setInputSource(sourceCloud);
    scia.setSearchMethodSource(tree2);
    scia.align (*ail_cloud);
    if (scia.hasConverged ())
        emit result(true,ail_cloud,scia.getFinalTransformation().cast<float>(),scia.getFitnessScore(),time.toc());
    else
        emit result(false,ail_cloud,Eigen::Matrix4f::Identity(),-1,time.toc());
}

void
Registration::SampleConsensusInitialAlignmentSHOT(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,const SHOTFeature::Ptr &targetFeature,const SHOTFeature::Ptr &sourceFeature,
                                              int nr_iterations,int ransac_iterations,double inlier_threshold,double distance_threshold,double euclidean_fitness_epsilon,
                                              int nr_samples, int k,float min_sample_distance)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::SampleConsensusInitialAlignment<PointXYZRGBN, PointXYZRGBN,pcl::SHOT352> scia;
    Cloud::Ptr ail_cloud(new Cloud);ail_cloud->copyInfo(sourceCloud);
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
    if (scia.hasConverged ())
        emit result(true,ail_cloud,scia.getFinalTransformation().cast<float>(),scia.getFitnessScore(),time.toc());
    else
        emit result(false,ail_cloud,Eigen::Matrix4f::Identity(),-1,time.toc());
}

void
Registration::FPCSInitialAlignment(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                   double distance_threshold,double euclidean_fitness_epsilon, float delta,float approx_overlap,float score_threshold,int nr_samples)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::registration::FPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN,PointXYZRGBN> fpcsi;
    Cloud::Ptr ail_cloud(new Cloud);ail_cloud->copyInfo(sourceCloud);
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
    if (fpcsi.hasConverged ())
        emit result(true,ail_cloud,fpcsi.getFinalTransformation().cast<float>(),fpcsi.getFitnessScore(),time.toc());
    else
        emit result(false,ail_cloud,Eigen::Matrix4f::Identity(),-1,time.toc());
}

void
Registration::KFPCSInitialAlignment(const Cloud::Ptr &targetCloud,const Cloud::Ptr &sourceCloud,int nr_iterations,int ransac_iterations,double inlier_threshold,
                                    double distance_threshold,double euclidean_fitness_epsilon, float delta,float approx_overlap,float score_threshold,
                                    int nr_samples,float upper_trl_boundary,float lower_trl_boundary,float lambda)
{
    pcl::console::TicToc time;
    time.tic ();
    pcl::registration::KFPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN,PointXYZRGBN> kfpcsi;
    Cloud::Ptr ail_cloud(new Cloud);ail_cloud->copyInfo(sourceCloud);
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
    if (kfpcsi.hasConverged ())
        emit result(true,ail_cloud,kfpcsi.getFinalTransformation().cast<float>(),kfpcsi.getFitnessScore(),time.toc());
    else
        emit result(false,ail_cloud,Eigen::Matrix4f::Identity(),-1,time.toc());
}


