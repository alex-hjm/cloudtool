#include "keypoints.h"

KeyPoints::KeyPoints()
{

}

CloudXYZRGBN::Ptr
KeyPoints::NarfKeyPoint(const CloudXYZRGBN::Ptr&cloud)
{
    time.tic();
    CloudXYZRGBN& point_cloud = *cloud;
    float angular_resolution = 0.5f;
    float support_size = 0.2f;
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    bool setUnseenToMaxRange = false;

    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                         point_cloud.sensor_origin_[1],
            point_cloud.sensor_origin_[2])) *
            Eigen::Affine3f (point_cloud.sensor_orientation_);

    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;

    pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    range_image.createFromPointCloud (point_cloud, pcl::deg2rad(angular_resolution), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                      scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges (far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange ();

    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute(keypoint_indices);

    CloudXYZRGBN::Ptr keypoints_ptr(new CloudXYZRGBN);
    keypoints_ptr->points.resize(keypoint_indices.points.size());
    for (size_t i=0; i<keypoint_indices.points.size (); ++i)
        keypoints_ptr->points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();
    tocTime=time.toc();
    return keypoints_ptr;
}

CloudXYZRGBN::Ptr
KeyPoints::SiftKeyPoint(const CloudXYZRGBN::Ptr &cloud ,int k,float r, float min_scale, int n_octaves,
                        int n_scales_per_octave, float min_contrast)
{
    time.tic();
    CloudN::Ptr cloud_normal(new CloudN);
    pcl::copyPointCloud(*cloud,*cloud_normal);
    CloudXYZRGBN::Ptr keypoints(new CloudXYZRGBN);
    pcl::SIFTKeypoint<PointN, PointXYZRGBN> sift;
    sift.setInputCloud(cloud_normal);
    pcl::search::KdTree<PointN>::Ptr tree(new pcl::search::KdTree<PointN>());
    sift.setSearchMethod(tree);
    sift.setKSearch(k);
    sift.setRadiusSearch(r);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.compute(*keypoints);
    tocTime=time.toc();
    return keypoints;
}

CloudXYZRGBN::Ptr
KeyPoints::HarrisKeyPoint(const CloudXYZRGBN::Ptr &cloud ,int k,float r,  int type,float r_keypoint,
                          float threshold,bool do_refine)
{
    time.tic();
    CloudXYZRGBN::Ptr Harris_keypoints(new CloudXYZRGBN);
    CloudXYZI::Ptr Harris_keypoints_xyzi (new CloudXYZI());
    pcl::HarrisKeypoint3D<PointXYZRGBN,PointXYZI,PointXYZRGBN> harris_detector;
    harris_detector.setInputCloud (cloud);
    harris_detector.setNormals(cloud);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>());
    harris_detector.setSearchMethod(tree);
    harris_detector.setKSearch(k);
    harris_detector.setRadiusSearch(r);
    harris_detector.setRadius(r_keypoint);
    harris_detector.setMethod(pcl::HarrisKeypoint3D<PointXYZRGBN,PointXYZI,PointXYZRGBN>::ResponseMethod(type));
    harris_detector.setRefine(do_refine);
    harris_detector.setThreshold(threshold);
    harris_detector.setNumberOfThreads(12);
    harris_detector.compute (*Harris_keypoints_xyzi);
    pcl::copyPointCloud(*Harris_keypoints_xyzi,*Harris_keypoints);
    tocTime=time.toc();
    return Harris_keypoints;
}

CloudXYZRGBN::Ptr KeyPoints::ISSKeypoint3D(const CloudXYZRGBN::Ptr &cloud, int k, float r, double salient_radius,
                                              double non_max_radius, double normal_radius, double border_radius, double gamma_21,
                                              double gamma_32, int min_neighbors, float angle)
{
    time.tic();
    CloudXYZRGBN::Ptr keypoints(new CloudXYZRGBN);
    pcl::ISSKeypoint3D<PointXYZRGBN,PointXYZRGBN,PointXYZRGBN>iss(salient_radius);
    iss.setInputCloud (cloud);
    iss.setNormals(cloud);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>());
    iss.setSearchMethod(tree);
    iss.setRadiusSearch(r);
    iss.setKSearch(k);
    iss.setNonMaxRadius(non_max_radius);
    iss.setNormalRadius(normal_radius);
    iss.setBorderRadius(border_radius);
    iss.setThreshold21(gamma_21);
    iss.setThreshold32(gamma_32);
    iss.setMinNeighbors(min_neighbors);
    iss.setAngleThreshold(angle);
    iss.compute(*keypoints);
    tocTime=time.toc();
    return keypoints;
}























