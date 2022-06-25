/**
 * @file features.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#include "modules/features.h"

#include <pcl/console/time.h>
#include <pcl/features/boundary.h>

#include <pcl/features/impl/3dsc.hpp>
#include <pcl/features/impl/board.hpp>
#include <pcl/features/impl/crh.hpp>
#include <pcl/features/impl/cvfh.hpp>
#include <pcl/features/impl/don.hpp>
#include <pcl/features/impl/esf.hpp>
#include <pcl/features/impl/flare.hpp>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/features/impl/fpfh_omp.hpp>
#include <pcl/features/impl/gasd.hpp>
#include <pcl/features/impl/grsd.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/impl/pfh.hpp>
#include <pcl/features/impl/rsd.hpp>
#include <pcl/features/impl/shot.hpp>
#include <pcl/features/impl/shot_lrf.hpp>
#include <pcl/features/impl/shot_lrf_omp.hpp>
#include <pcl/features/impl/shot_omp.hpp>
#include <pcl/features/impl/usc.hpp>
#include <pcl/features/impl/vfh.hpp>

namespace ct
{

    Box Features::boundingBoxAABB(const Cloud::Ptr& cloud)
    {
        PointXYZRGBN min, max;
        pcl::getMinMax3D(*cloud, min, max);
        Eigen::Vector3f cloud_center =
            0.5f * (min.getVector3fMap() + max.getVector3fMap());
        Eigen::Vector3f whd;
        whd = max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f affine = pcl::getTransformation(
            cloud_center[0], cloud_center[1], cloud_center[2], 0, 0, 0);
        return { whd(0), whd(1), whd(2), affine, cloud_center,
                Eigen::Quaternionf(Eigen::Matrix3f::Identity()) };
    }

    Box Features::boundingBoxOBB(const Cloud::Ptr& cloud)
    {
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cloud, pcaCentroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();                 // feature vector
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); // Correct the verticality between main directions
        eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
        eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f transform_inv = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
        transform.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>());
        transform_inv = transform.inverse();
        Cloud::Ptr transformedCloud(new Cloud);
        transformPointCloud(*cloud, *transformedCloud, transform);
        PointXYZRGBN min, max;
        Eigen::Vector3f cloud_center, tcloud_center;
        pcl::getMinMax3D(*transformedCloud, min, max);
        cloud_center = 0.5f * (min.getVector3fMap() + max.getVector3fMap());
        Eigen::Vector3f whd = max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f transform_inv_aff(transform_inv);
        pcl::transformPoint(cloud_center, tcloud_center, transform_inv_aff);
        return { whd(0), whd(1), whd(2), transform_inv_aff, tcloud_center,
                Eigen::Quaternionf(transform_inv.block<3, 3>(0, 0)) };
    }

    Box Features::boundingBoxAdjust(const Cloud::Ptr& cloud,
                                    const Eigen::Affine3f& t)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform = t.matrix();
        Eigen::Matrix4f transform_inv = Eigen::Matrix4f::Identity();
        transform_inv = transform.inverse();
        Cloud::Ptr transformedCloud(new Cloud);
        transformPointCloud(*cloud, *transformedCloud, transform);
        PointXYZRGBN min, max;
        Eigen::Vector3f cloud_center, tcloud_center;
        pcl::getMinMax3D(*transformedCloud, min, max);
        cloud_center = 0.5f * (min.getVector3fMap() + max.getVector3fMap());
        Eigen::Vector3f whd = max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f transform_inv_aff(transform_inv);
        pcl::transformPoint(cloud_center, tcloud_center, transform_inv_aff);
        return { whd(0), whd(1), whd(2), transform_inv_aff,
                tcloud_center, Eigen::Quaternionf(transform_inv.block<3, 3>(0, 0)) };
    }

    void Features::NormalEstimation(float vpx, float vpy, float vpz)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_with_normals = cloud_->makeShared();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::NormalEstimationOMP<PointXYZRGBN, PointXYZRGBN> est_normal;
        est_normal.setInputCloud(cloud_with_normals);
        est_normal.setSearchMethod(tree);
        est_normal.setKSearch(k_);
        est_normal.setRadiusSearch(radius_);
        est_normal.setNumberOfThreads(12);
        est_normal.setViewPoint(vpx, vpy, vpz);
        est_normal.compute(*cloud_with_normals);
        emit normalsResult(cloud_with_normals, time.toc());
    }

    void Features::DifferenceOfNormalsEstimation(const Cloud::Ptr& small_normal, const Cloud::Ptr& large_normal)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        Cloud::Ptr feature = cloud_->makeShared();

        pcl::DifferenceOfNormalsEstimation<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setNormalScaleSmall(small_normal);
        est.setNormalScaleLarge(large_normal);
        est.computeFeature(*feature);
        emit donResult(feature, time.toc());
    }

    void Features::BoundaryEstimation(float angle)
    {
        TicToc time;
        time.tic();
        pcl::PointCloud<pcl::Boundary> boundaries;
        Cloud::Ptr cloud_boundary(new Cloud);
        cloud_boundary->setId(cloud_->id());
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::BoundaryEstimation<PointXYZRGBN, PointXYZRGBN, pcl::Boundary> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setInputNormals(cloud_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setAngleThreshold(pcl::deg2rad(angle));
        est.compute(boundaries);

        for (int i = 0; i < cloud_->points.size(); i++)
        {
            if (boundaries[i].boundary_point > 0)
            {
                cloud_boundary->push_back(cloud_->points[i]);
            }
        }
        emit boundaryResult(cloud_boundary, time.toc());
    }

    void Features::PFHEstimation()
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->pfh.reset(new PFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::PFHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::PFHSignature125> pfh;
        pfh.setSearchMethod(tree);
        pfh.setSearchSurface(surface_);
        pfh.setInputCloud(cloud_);
        pfh.setInputNormals(cloud_);
        pfh.setKSearch(k_);
        pfh.setRadiusSearch(radius_);
        pfh.compute(*feature->pfh);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::FPFHEstimation()
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->fpfh.reset(new FPFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::FPFHEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(cloud_);
        fpfh.setInputNormals(cloud_);
        fpfh.setSearchSurface(surface_);
        fpfh.setSearchMethod(tree);
        fpfh.setKSearch(k_);
        fpfh.setRadiusSearch(radius_);
        fpfh.setNumberOfThreads(12);
        fpfh.compute(*feature->fpfh);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::VFHEstimation(const Eigen::Vector3f& dir)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->vfh.reset(new VFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::VFHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::VFHSignature308> vfh;
        vfh.setSearchMethod(tree);
        vfh.setInputCloud(cloud_);
        vfh.setInputNormals(cloud_);
        vfh.setSearchSurface(surface_);
        vfh.setKSearch(k_);
        vfh.setRadiusSearch(radius_);
        vfh.setViewPoint(dir[0], dir[1], dir[2]);
        vfh.compute(*feature->vfh);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::ESFEstimation()
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->esf.reset(new ESFFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::ESFEstimation<PointXYZRGBN, pcl::ESFSignature640> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.compute(*feature->esf);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::GASDEstimation(const Eigen::Vector3f& dir, int shgs, int shs, int interp)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->gasd.reset(new GASDFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::GASDEstimation<PointXYZRGBN, pcl::GASDSignature512> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setViewDirection(dir);
        est.setShapeHalfGridSize(shgs);
        est.setShapeHistsSize(shs);
        est.setShapeHistsInterpMethod(pcl::HistogramInterpolationMethod(interp));
        est.compute(*feature->gasd);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::GASDColorEstimation(const Eigen::Vector3f& dir, int shgs, int shs, int interp,
                                       int chgs, int chs, int cinterp)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->gasdc.reset(new GASDCFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::GASDColorEstimation<PointXYZRGBN, pcl::GASDSignature984> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setViewDirection(dir);
        est.setShapeHalfGridSize(shgs);
        est.setShapeHistsSize(shs);
        est.setShapeHistsInterpMethod(pcl::HistogramInterpolationMethod(interp));
        est.setColorHalfGridSize(chgs);
        est.setColorHistsSize(chs);
        est.setColorHistsInterpMethod(pcl::HistogramInterpolationMethod(interp));
        est.compute(*feature->gasdc);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::RSDEstimation(int nr_subdiv, double plane_radius)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->rsd.reset(new RSDFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::RSDEstimation<PointXYZRGBN, PointXYZRGBN, pcl::PrincipalRadiiRSD> est;
        est.setSearchMethod(tree);
        est.setSearchSurface(surface_);
        est.setInputCloud(cloud_);
        est.setInputNormals(cloud_);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setNrSubdivisions(nr_subdiv);
        est.setPlaneRadius(plane_radius);
        //est.setSaveHistograms(true);
        est.compute(*feature->rsd);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::GRSDEstimation()
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->grsd.reset(new GRSDFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::GRSDEstimation<PointXYZRGBN, PointXYZRGBN, pcl::GRSDSignature21> est;
        est.setInputCloud(cloud_);
        est.setInputNormals(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.compute(*feature->grsd);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::CRHEstimation(const Eigen::Vector3f& dir)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->crh.reset(new CRHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::CRHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::Histogram<90>> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setInputNormals(cloud_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setViewPoint(dir[0], dir[1], dir[2]);
        est.compute(*feature->crh);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::CVFHEstimation(const Eigen::Vector3f& dir, float radius_normals,
                                  float d1, float d2, float d3, int min, bool normalize)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->vfh.reset(new VFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::CVFHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::VFHSignature308> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setInputNormals(cloud_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setViewPoint(dir[0], dir[1], dir[2]);
        est.setRadiusNormals(radius_normals);
        est.setClusterTolerance(d1);
        est.setEPSAngleThreshold(d2);
        est.setCurvatureThreshold(d3);
        est.setMinPoints(min);
        est.setNormalizeBins(normalize);
        est.compute(*feature->vfh);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::ShapeContext3DEstimation(double min_radius, double radius)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->sc3d.reset(new SC3DFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::ShapeContext3DEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ShapeContext1980> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setInputNormals(cloud_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setMinimalRadius(min_radius);
        est.setPointDensityRadius(radius);
        est.compute(*feature->sc3d);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::SHOTEstimation(const ReferenceFrame::Ptr& lrf, float radius)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->shot.reset(new SHOTFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::SHOTEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::SHOT352> shot;
        shot.setSearchMethod(tree);
        shot.setSearchSurface(surface_);
        shot.setInputCloud(cloud_);
        shot.setInputNormals(cloud_);
        shot.setKSearch(k_);
        shot.setRadiusSearch(radius_);
        shot.setNumberOfThreads(12);
        shot.setLRFRadius(radius);
        shot.setInputReferenceFrames(lrf);
        shot.compute(*feature->shot);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::SHOTColorEstimation(const ReferenceFrame::Ptr& lrf, float radius)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->shotc.reset(new SHOTCFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::SHOTColorEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::SHOT1344> shot;
        shot.setSearchMethod(tree);
        shot.setSearchSurface(surface_);
        shot.setInputCloud(cloud_);
        shot.setInputNormals(cloud_);
        shot.setKSearch(k_);
        shot.setRadiusSearch(radius_);
        shot.setNumberOfThreads(12);
        shot.setLRFRadius(radius);
        shot.setInputReferenceFrames(lrf);
        shot.compute(*feature->shotc);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::UniqueShapeContext(const ReferenceFrame::Ptr& lrf, double min_radius,
                                      double pt_radius, double loc_radius)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->usc.reset(new USCFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::UniqueShapeContext<PointXYZRGBN, pcl::UniqueShapeContext1960, pcl::ReferenceFrame>
            est;
        est.setSearchMethod(tree);
        est.setSearchSurface(surface_);
        est.setInputCloud(cloud_);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setInputReferenceFrames(lrf);
        est.setMinimalRadius(min_radius);
        est.setPointDensityRadius(pt_radius);
        est.setLocalRadius(loc_radius);
        est.compute(*feature->usc);
        emit featureResult(cloud_->id(), feature, time.toc());
    }
    
    void Features::BOARDLocalReferenceFrameEstimation(float radius, bool find_holes, float margin_thresh, int size,
                                                      float prob_thresh, float steep_thresh)
    {
        TicToc time;
        time.tic();
        ReferenceFrame::Ptr feature(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::BOARDLocalReferenceFrameEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ReferenceFrame> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setInputNormals(cloud_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setTangentRadius(radius);
        est.setFindHoles(find_holes);
        est.setMarginThresh(margin_thresh);
        est.setCheckMarginArraySize(size);
        est.setHoleSizeProbThresh(prob_thresh);
        est.setSteepThresh(steep_thresh);
        est.compute(*feature);
        emit lrfResult(cloud_->id(), feature, time.toc());
    }

    void Features::FLARELocalReferenceFrameEstimation(float radius, float margin_thresh, int min_neighbors_for_normal_axis,
                                                      int min_neighbors_for_tangent_axis)
    {
        TicToc time;
        time.tic();
        ReferenceFrame::Ptr feature(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::FLARELocalReferenceFrameEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ReferenceFrame> est;
        est.setInputCloud(cloud_);
        est.setInputNormals(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setTangentRadius(radius);
        est.setMarginThresh(margin_thresh);
        est.setMinNeighboursForNormalAxis(min_neighbors_for_normal_axis);
        est.setMinNeighboursForTangentAxis(min_neighbors_for_tangent_axis);
        est.compute(*feature);
        emit lrfResult(cloud_->id(), feature, time.toc());
    }

    void Features::SHOTLocalReferenceFrameEstimation()
    {
        TicToc time;
        time.tic();
        ReferenceFrame::Ptr feature(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::SHOTLocalReferenceFrameEstimationOMP<PointXYZRGBN, pcl::ReferenceFrame>
            est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setNumberOfThreads(12);
        est.compute(*feature);
        emit lrfResult(cloud_->id(), feature, time.toc());
    }

} // namespace ct
