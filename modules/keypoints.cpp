/**
 * @file keypoints.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#include "modules/keypoints.h"

#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/trajkovic_3d.h>

namespace ct
{
  void Keypoints::HarrisKeypoint3D(int response_method, float radius, float threshold,
                                   bool non_maxima, bool do_refine)
  {
    TicToc time;
    time.tic();
    pcl::PointCloud<PointXYZI>::Ptr keypoints_temp(
      new pcl::PointCloud<PointXYZI>);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

    pcl::HarrisKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN> est;
    est.setInputCloud(cloud_);
    est.setSearchSurface(surface_);
    est.setNormals(cloud_);
    est.setSearchMethod(tree);
    est.setKSearch(k_);
    est.setRadiusSearch(radius_);
    est.setMethod(
      pcl::HarrisKeypoint3D<PointXYZRGBN, PointXYZI,
      PointXYZRGBN>::ResponseMethod(response_method + 1));
    est.setRadius(radius);
    est.setThreshold(threshold);
    est.setNonMaxSupression(non_maxima);
    est.setRefine(do_refine);
    est.setNumberOfThreads(12);
    est.compute(*keypoints_temp);
    Cloud::Ptr keypoints(new Cloud);
    copyPointCloud(*keypoints_temp, *keypoints);
    emit keypointsResult(keypoints, time.toc());
  }

  void Keypoints::ISSKeypoint3D(double salient_radius, double non_max_radius,
                                double normal_radius, double border_radius,
                                double gamma_21, double gamma_32,
                                int min_neighbors, float angle)
  {
    TicToc time;
    time.tic();
    Cloud::Ptr keypoints(new Cloud);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

    pcl::ISSKeypoint3D<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN> est;
    est.setInputCloud(cloud_);
    est.setSearchSurface(surface_);
    est.setNormals(cloud_);
    est.setSearchMethod(tree);
    est.setKSearch(k_);
    est.setRadiusSearch(radius_);
    est.setSalientRadius(salient_radius);
    est.setNonMaxRadius(non_max_radius);
    est.setNormalRadius(normal_radius);
    est.setBorderRadius(border_radius);
    est.setThreshold21(gamma_21);
    est.setThreshold32(gamma_32);
    est.setMinNeighbors(min_neighbors);
    est.setAngleThreshold((angle / 180) * M_PI);
    est.setNumberOfThreads(12);
    est.compute(*keypoints);
    emit keypointsResult(keypoints, time.toc());
  }

  void Keypoints::SIFTKeypoint(float min_scale, int nr_octaves,
                               int nr_scales_per_octave, float min_contrast)
  {
    TicToc time;
    time.tic();
    Cloud::Ptr keypoints(new Cloud);
    pcl::PointCloud<PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<PointXYZI>);
    pcl::copyPointCloud(*cloud_, *cloud_xyzi);
    pcl::search::KdTree<PointXYZI>::Ptr tree(new pcl::search::KdTree<PointXYZI>);

    pcl::SIFTKeypoint<PointXYZI, PointXYZRGBN> est;
    est.setInputCloud(cloud_xyzi);
    est.setSearchSurface(cloud_xyzi);
    est.setSearchMethod(tree);
    est.setKSearch(k_);
    est.setRadiusSearch(radius_);
    est.setScales(min_scale, nr_octaves, nr_scales_per_octave);
    if (min_contrast >= 0) est.setMinimumContrast(min_contrast);
    est.compute(*keypoints);
    emit keypointsResult(keypoints, time.toc());
  }

  void Keypoints::TrajkovicKeypoint3D(int compute_method, int window_size,
                                      float first_threshold,
                                      float second_threshold)
  {
    TicToc time;
    time.tic();
    pcl::PointCloud<PointXYZI>::Ptr keypoints_temp(
      new pcl::PointCloud<PointXYZI>);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

    pcl::TrajkovicKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN> est;
    est.setInputCloud(cloud_);
    est.setSearchSurface(surface_);
    est.setNormals(cloud_);
    est.setSearchMethod(tree);
    est.setKSearch(k_);
    est.setRadiusSearch(radius_);
    est.setMethod(
      pcl::TrajkovicKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN>::
      ComputationMethod(compute_method));
    est.setWindowSize(window_size);
    est.setFirstThreshold(first_threshold);
    est.setSecondThreshold(second_threshold);
    est.setNumberOfThreads(12);
    est.compute(*keypoints_temp);
    Cloud::Ptr keypoints(new Cloud);
    copyPointCloud(*keypoints_temp, *keypoints);
    emit keypointsResult(keypoints, time.toc());
  }

}  // namespace pca
