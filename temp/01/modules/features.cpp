#include "features.h"

Features::Features(QObject *parent) : QObject(parent)
{

}

void
Features::normalEstimation(const Cloud::Ptr &cloud, int K,double R,const Eigen::Vector3f &viewpoint)
{
    pcl::console::TicToc time;
    time.tic();
    pcl::NormalEstimationOMP<PointXYZRGBN, PointXYZRGBN> est_normal;
    Cloud::Ptr cloud_with_normals=cloud->makeShared();
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    est_normal.setViewPoint(viewpoint[0],viewpoint[1],viewpoint[2]);
    est_normal.setInputCloud(cloud_with_normals);
    est_normal.setSearchMethod(tree);
    est_normal.setKSearch(K);
    est_normal.setRadiusSearch(R);
    est_normal.setNumberOfThreads(12);
    est_normal.compute(*cloud_with_normals);
    emit result(cloud_with_normals,time.toc());
}

Box
Features::boundingBoxAABB(const Cloud::Ptr &cloud)
{
    PointXYZRGBN min, max;
    pcl::getMinMax3D(*cloud, min, max);
    Eigen::Vector3f cloud_center= 0.5f*(min.getVector3fMap() + max.getVector3fMap());
    Eigen::Vector3f whd;
    whd = max.getVector3fMap() - min.getVector3fMap();
    Eigen::Affine3f affine=pcl::getTransformation(cloud_center[0],cloud_center[1],cloud_center[2],0,0,0);
    return {whd(0),whd(1),whd(2),affine,cloud_center,Eigen::Quaternionf(Eigen::Matrix3f::Identity())};
}

Box
Features::boundingBoxOBB(const Cloud::Ptr &cloud)
{
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
    Cloud::Ptr transformedCloud(new Cloud);
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

Box
Features::boundingBoxAdjust(const Cloud::Ptr &cloud, const Eigen::Affine3f &t)
{
    Eigen::Matrix4f transform=Eigen::Matrix4f::Identity();
    transform=t.matrix();
    Eigen::Matrix4f transform_inv=Eigen::Matrix4f::Identity();
    transform_inv=transform.inverse();
    Cloud::Ptr transformedCloud(new Cloud);
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

PFHFeature::Ptr
Features::PFHEstimation(const Cloud::Ptr& cloud,int k, double radius)
{
    PFHFeature::Ptr feature(new PFHFeature);
    pcl::PFHEstimation<PointXYZRGBN,PointXYZRGBN,pcl::PFHSignature125> pfh;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    pfh.setSearchMethod(tree);
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(cloud);
    pfh.setKSearch(k);
    pfh.setRadiusSearch(radius);
    pfh.compute(*feature);
    return feature;
}

FPFHFeature::Ptr
Features::FPFHEstimation(const Cloud::Ptr& cloud,int k, double radius)
{
    FPFHFeature::Ptr feature(new FPFHFeature);
    pcl::FPFHEstimationOMP<PointXYZRGBN,PointXYZRGBN,pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    fpfh.setSearchMethod(tree);
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(cloud);
    fpfh.setKSearch(k);
    fpfh.setRadiusSearch(radius);
    fpfh.setNumberOfThreads(12);
    fpfh.compute(*feature);
    return feature;
}

VFHFeature::Ptr
Features::VFHEstimation(const Cloud::Ptr& cloud,int k,double radius)
{
    VFHFeature::Ptr feature(new VFHFeature);
    pcl::VFHEstimation<PointXYZRGBN,PointXYZRGBN,pcl::VFHSignature308> vfh;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    vfh.setSearchMethod(tree);
    vfh.setInputCloud(cloud);
    vfh.setInputNormals(cloud);
    vfh.setKSearch(k);
    vfh.setRadiusSearch(radius);
    vfh.compute(*feature);
    return feature;
}

SHOTFeature::Ptr
Features::SHOTEstimation(const Cloud::Ptr& cloud,int k, double radius)
{
    SHOTFeature::Ptr feature(new SHOTFeature);
    pcl::SHOTEstimationOMP<PointXYZRGBN,PointXYZRGBN,pcl::SHOT352> shot;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    shot.setSearchMethod(tree);
    shot.setInputCloud(cloud);
    shot.setInputNormals(cloud);
    shot.setKSearch(k);
    shot.setRadiusSearch(radius);
    shot.setNumberOfThreads(12);
    shot.compute(*feature);
    return feature;
}


