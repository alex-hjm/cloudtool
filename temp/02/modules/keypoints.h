#ifndef KEYPOINTS_H
#define KEYPOINTS_H

#include <pcl/common/angles.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/smoothed_surfaces_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/range_image/range_image.h>
#ifdef _WIN32
#include <pcl/features/impl/normal_3d.hpp>
#endif
#include "src/common/customtype.h"
class KeyPoints
{
public:
    KeyPoints();

    float tocTime;
    CloudXYZRGBN::Ptr NarfKeyPoint(const CloudXYZRGBN::Ptr & cloud);
    CloudXYZRGBN::Ptr SiftKeyPoint(const CloudXYZRGBN::Ptr &cloud ,int k,float r, float min_scale, int n_octaves,
                                   int n_scales_per_octave, float min_contrast);
    CloudXYZRGBN::Ptr HarrisKeyPoint(const CloudXYZRGBN::Ptr &cloud ,int k,float r,int type,float r_keypoint,
                                     float threshold,bool do_refine);
    CloudXYZRGBN::Ptr ISSKeypoint3D(const CloudXYZRGBN::Ptr &cloud ,int k,float r,double salient_radius,double non_max_radius,
                                    double normal_radius,double border_radius,double gamma_21,double gamma_32,int min_neighbors,
                                    float angle);
private:
     pcl::console::TicToc time;
};

#endif // KEYPOINTSMODULE_H

