#include "features.h"

Features::Features()
{

}

CloudXYZRGBN::Ptr
Features::NormalEstimation(const CloudXYZRGBN::Ptr &cloud, int K,double R,const Eigen::Vector3f &viewpoint)
{
    time.tic();
    pcl::NormalEstimationOMP<PointXYZRGBN, Normal> est_normal;
    CloudNormal::Ptr normals(new CloudNormal);
    CloudXYZRGBN::Ptr cloudwithnormals(new CloudXYZRGBN);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    est_normal.setViewPoint(viewpoint[0],viewpoint[1],viewpoint[2]);
    est_normal.setInputCloud(cloud);
    est_normal.setSearchMethod(tree);
    est_normal.setKSearch(K);
    est_normal.setRadiusSearch(R);
    est_normal.setNumberOfThreads(12);
    est_normal.compute(*normals);
    pcl::concatenateFields(*cloud,*normals,*cloudwithnormals);
    tocTime=time.toc();
    return cloudwithnormals;
}

BoundingBox
Features::BoundingBoxAABB(const CloudXYZRGBN::Ptr &cloud)
{
    time.tic();
    PointXYZRGBN min, max;
    pcl::getMinMax3D(*cloud, min, max);
    Eigen::Vector3f cloud_center= 0.5f*(min.getVector3fMap() + max.getVector3fMap());
    Eigen::Vector3f whd;
    whd = max.getVector3fMap() - min.getVector3fMap();
    Eigen::Affine3f affine=Eigen::Affine3f::Identity();
    affine=pcl::getTransformation(cloud_center[0],cloud_center[1],cloud_center[2],0,0,0);
    tocTime=time.toc();
    return {whd(0),whd(1),whd(2),affine,cloud_center,Eigen::Quaternionf(Eigen::Matrix3f::Identity())};
}

BoundingBox
Features::BoundingBoxOBB(const CloudXYZRGBN::Ptr &cloud)
{
    time.tic();
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();//feature vector
    //Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();//feature values

    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //Correct the verticality between main directions
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

    Eigen::Matrix4f transform=Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_inv=Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    transform.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());
    transform_inv=transform.inverse();

    CloudXYZRGBN::Ptr transformedCloud(new CloudXYZRGBN);
    pcl::transformPointCloud(*cloud, *transformedCloud, transform);

    PointXYZRGBN min, max;
    Eigen::Vector3f cloud_center, tcloud_center;
    pcl::getMinMax3D(*transformedCloud, min, max);
    cloud_center= 0.5f*(min.getVector3fMap() + max.getVector3fMap());
    Eigen::Vector3f whd= max.getVector3fMap() - min.getVector3fMap();

    Eigen::Affine3f transform_inv_aff(transform_inv);
    pcl::transformPoint(cloud_center, tcloud_center, transform_inv_aff);
    tocTime=time.toc();
    return {whd(0),whd(1),whd(2),transform_inv_aff,tcloud_center,Eigen::Quaternionf(transform_inv.block<3, 3>(0, 0))};

}

BoundingBox
Features::BoundingBoxAdjust(const CloudXYZRGBN::Ptr &cloud, const Eigen::Affine3f &t)
{
    Eigen::Matrix4f transform=Eigen::Matrix4f::Identity();
    transform=t.matrix();
    Eigen::Matrix4f transform_inv=Eigen::Matrix4f::Identity();
    transform_inv=transform.inverse();

    CloudXYZRGBN::Ptr transformedCloud(new CloudXYZRGBN);
    pcl::transformPointCloud(*cloud, *transformedCloud, transform);

    PointXYZRGBN min, max;
    Eigen::Vector3f cloud_center, tcloud_center;
    pcl::getMinMax3D(*transformedCloud, min, max);
    cloud_center= 0.5f*(min.getVector3fMap() + max.getVector3fMap());
    Eigen::Vector3f whd= max.getVector3fMap() - min.getVector3fMap();

    Eigen::Affine3f transform_inv_aff(transform_inv);
    pcl::transformPoint(cloud_center, tcloud_center, transform_inv_aff);

    return {whd(0),whd(1),whd(2),transform_inv_aff,tcloud_center,Eigen::Quaternionf(transform_inv.block<3, 3>(0, 0))};
}

ReferenceFrame::Ptr
Features::BOARDLocalReferenceFrameEstimation(const CloudXYZRGBN::Ptr &keypoints, const CloudXYZRGBN::Ptr &cloud, int k, double radius,
                                             float tangent_radius, float margin_thresh, bool find_holes)
{
    ReferenceFrame::Ptr lrf(new ReferenceFrame);
    pcl::BOARDLocalReferenceFrameEstimation<PointXYZRGBN,PointXYZRGBN,pcl::ReferenceFrame> board;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    board.setSearchMethod(tree);
    board.setInputCloud(keypoints);
    board.setInputNormals(cloud);
    board.setSearchSurface(cloud);
    board.setKSearch(k);
    board.setRadiusSearch(radius);
    board.setTangentRadius(tangent_radius);
    board.setMarginThresh(margin_thresh);
    board.setFindHoles(find_holes);
    board.compute(*lrf);
    return lrf;
}

ReferenceFrame::Ptr
Features::FLARELocalReferenceFrameEstimation(const CloudXYZRGBN::Ptr &keypoints, const CloudXYZRGBN::Ptr &cloud, int k, double radius,
                                             float tangent_radius, float margin_thresh)
{
    ReferenceFrame::Ptr lrf(new ReferenceFrame);
    pcl::FLARELocalReferenceFrameEstimation<PointXYZRGBN,PointXYZRGBN,pcl::ReferenceFrame> flare;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree1(new pcl::search::KdTree<PointXYZRGBN>);
    flare.setSearchMethod(tree1);
    flare.setInputCloud(keypoints);
    flare.setInputNormals(cloud);
    flare.setSearchSurface(cloud);
    flare.setKSearch(k);
    flare.setRadiusSearch(radius);
    flare.setTangentRadius(tangent_radius);
    flare.setMarginThresh(margin_thresh);
    flare.compute(*lrf);
    return lrf;
}

ReferenceFrame::Ptr
Features::SHOTLocalReferenceFrameEstimation(const CloudXYZRGBN::Ptr &keypoints, const CloudXYZRGBN::Ptr &cloud, int k, double radius)
{
    ReferenceFrame::Ptr lrf(new ReferenceFrame);
    pcl::SHOTLocalReferenceFrameEstimationOMP<PointXYZRGBN,pcl::ReferenceFrame> shot;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    shot.setSearchMethod(tree);
    shot.setInputCloud(keypoints);
    shot.setSearchSurface(cloud);
    shot.setKSearch(k);
    shot.setRadiusSearch(radius);
    shot.setNumberOfThreads(12);
    shot.compute(*lrf);
    return lrf;
}


PFHFeature::Ptr
Features::PFHEstimation(const CloudXYZRGBN::Ptr &keypoints,const CloudXYZRGBN::Ptr& cloud,int k, double radius)
{
    time.tic();
    PFHFeature::Ptr feature(new PFHFeature);
    pcl::PFHEstimation<PointXYZRGBN,PointXYZRGBN,pcl::PFHSignature125> pfh;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    pfh.setSearchMethod(tree);
    pfh.setInputCloud(keypoints);
    pfh.setInputNormals(cloud);
    pfh.setKSearch(k);
    pfh.setRadiusSearch(radius);
    pfh.setSearchSurface(cloud);
    pfh.compute(*feature);
    tocTime=time.toc();
    return feature;
}

FPFHFeature::Ptr
Features::FPFHEstimation(const CloudXYZRGBN::Ptr &keypoints,const CloudXYZRGBN::Ptr& cloud,int k, double radius)
{
    time.tic();
    FPFHFeature::Ptr feature(new FPFHFeature);
    pcl::FPFHEstimationOMP<PointXYZRGBN,PointXYZRGBN,pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    fpfh.setSearchMethod(tree);
    fpfh.setInputCloud(keypoints);
    fpfh.setInputNormals(cloud);
    fpfh.setKSearch(k);
    fpfh.setRadiusSearch(radius);
    fpfh.setSearchSurface(cloud);
    fpfh.setNumberOfThreads(12);
    fpfh.compute(*feature);
    tocTime=time.toc();
    return feature;
}

VFHFeature::Ptr
Features::VFHEstimation(const CloudXYZRGBN::Ptr&keypoints,const CloudXYZRGBN::Ptr& cloud,int k,double radius)
{
    time.tic();
    VFHFeature::Ptr feature(new VFHFeature);
    pcl::VFHEstimation<PointXYZRGBN,PointXYZRGBN,pcl::VFHSignature308> vfh;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    vfh.setSearchMethod(tree);
    vfh.setInputCloud(keypoints);
    vfh.setInputNormals(cloud);
    vfh.setKSearch(k);
    vfh.setRadiusSearch(radius);
    vfh.setSearchSurface(cloud);
    vfh.compute(*feature);
    tocTime=time.toc();
    return feature;
}

SHOTFeature::Ptr
Features::SHOTEstimation(const CloudXYZRGBN::Ptr &keypoints,const CloudXYZRGBN::Ptr& cloud,int k, double radius)
{
    time.tic();
    SHOTFeature::Ptr feature(new SHOTFeature);
    pcl::SHOTEstimationOMP<PointXYZRGBN,PointXYZRGBN,pcl::SHOT352> shot;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    shot.setSearchMethod(tree);
    shot.setInputCloud(keypoints);
    shot.setInputNormals(cloud);
    shot.setKSearch(k);
    shot.setRadiusSearch(radius);
    shot.setSearchSurface(cloud);
    shot.setNumberOfThreads(12);
    shot.compute(*feature);
    tocTime=time.toc();
    return feature;
}




