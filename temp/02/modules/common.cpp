#include "common.h"

Common::Common()
{

}

CloudXYZRGBN::Ptr Common::setColor(const CloudXYZRGBN::Ptr &cloud, int r, int g, int b)
{
    CloudXYZRGBN::Ptr  colorCloud(new CloudXYZRGBN);
    *colorCloud=*cloud;
    std::uint32_t rgb = ((std::uint32_t)r<< 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    for (size_t j=0;j<colorCloud->points.size();j++) {
        colorCloud->points[j].rgb=*reinterpret_cast<float*>(&rgb);
    }
    return colorCloud;
}

CloudXYZRGBN::Ptr Common::setColor(const CloudXYZRGBN::Ptr &cloud, string aixs)
{
    CloudXYZRGBN::Ptr  colorCloud(new CloudXYZRGBN);
    *colorCloud=*cloud;
    float max=-FLT_MAX;
    float min=FLT_MAX;
    float fRed = 0.f;
    float fGreen = 0.f;
    float fBlue = 0.f;
    constexpr float range = 2.f / 3.f;
    constexpr uint8_t colorMax = std::numeric_limits<uint8_t>::max();

    PointXYZRGBN minPt, maxPt;
    pcl::getMinMax3D (*colorCloud, minPt, maxPt);

    if (aixs=="x") {
        max = maxPt.x;
        min = minPt.x;
        for (size_t i = 0; i < colorCloud->points.size(); i++)
        {
            float hue = (max-colorCloud->points[i].x) / static_cast<float>(max - min);
            hue *= range;
            hue = range - hue;
            tool.ConvertHSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
            colorCloud->points[i].r = static_cast<uint8_t>(fRed * colorMax);
            colorCloud->points[i].g = static_cast<uint8_t>(fGreen * colorMax);
            colorCloud->points[i].b = static_cast<uint8_t>(fBlue * colorMax);
        }
    }
    else if (aixs=="y") {
        max = maxPt.y;
        min = minPt.y;
        for (size_t i = 0; i < colorCloud->points.size(); i++)
        {
            float hue = (max-colorCloud->points[i].y) / static_cast<float>(max - min);
            hue *= range;
            hue = range - hue;
            tool.ConvertHSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
            colorCloud->points[i].r = static_cast<uint8_t>(fRed * colorMax);
            colorCloud->points[i].g = static_cast<uint8_t>(fGreen * colorMax);
            colorCloud->points[i].b = static_cast<uint8_t>(fBlue * colorMax);
        }
    }
    else if (aixs=="z") {
        max = maxPt.z;
        min = minPt.z;
        for (size_t i = 0; i < colorCloud->points.size(); i++)
        {
            float hue = (max-colorCloud->points[i].z) / static_cast<float>(max - min);
            hue *= range;
            hue = range - hue;
            tool.ConvertHSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
            colorCloud->points[i].r = static_cast<uint8_t>(fRed * colorMax);
            colorCloud->points[i].g = static_cast<uint8_t>(fGreen * colorMax);
            colorCloud->points[i].b = static_cast<uint8_t>(fBlue * colorMax);
        }
    }
    return colorCloud;
}

CloudXYZRGBN::Ptr Common::ScaleCloud(const CloudXYZRGBN::Ptr &cloud, const Eigen::Vector3f &center,
                                           double scaleX,double scaleY,double scaleZ,const bool &origin)
{
    Eigen::Affine3f Affine;
    CloudXYZRGBN::Ptr  scaledCloud(new CloudXYZRGBN);
    *scaledCloud=*cloud;
    for (int j=0;j<cloud->points.size();j++) {
        scaledCloud->points[j].x=center[0]+scaleX*(cloud->points[j].x-center[0]);
        scaledCloud->points[j].y=center[1]+scaleY*(cloud->points[j].y-center[1]);
        scaledCloud->points[j].z=center[2]+scaleZ*(cloud->points[j].z-center[2]);
    }
    if(origin) {
        pcl::getTransformation(center[0]*scaleX-center[0],center[1]*scaleY-center[1],center[2]*scaleZ-center[2],0,0,0,Affine);
        pcl::transformPointCloud(*scaledCloud,*scaledCloud,Affine);
    }
    return scaledCloud;
}

std::vector<CloudXYZRGBN::Ptr> Common::SelectCloud(const std::vector<Cloud> &clouds,int feature,int operation,float min,float max)
{
    std::vector<CloudXYZRGBN::Ptr> finalClouds;
    for (size_t i=0;i<clouds.size();i++) {
        switch (feature) {
        case 0://num_points
            if (operation==0) {
                if(clouds[i].size()>=min&&clouds[i].size()<=max) {
                    finalClouds.push_back(clouds[i].cloud);
                }
            } else {
                if((clouds[i].size()<min)| (clouds[i].size()>max)) {
                    finalClouds.push_back(clouds[i].cloud);
                }
            }
            break;
        case 1://volume
            if (operation==0) {
                if(clouds[i].volume()>=min&&clouds[i].volume()<=max) {
                    finalClouds.push_back(clouds[i].cloud);
                }
            } else {
                if((clouds[i].volume()<min)|(clouds[i].volume()>max)) {
                    finalClouds.push_back(clouds[i].cloud);
                }
            }
            break;
        case 2://Center_x
            if (operation==0) {
                if(clouds[i].center()[0]>=min&&clouds[i].center()[0]<=max) {
                    finalClouds.push_back(clouds[i].cloud);
                }
            } else {
                if((clouds[i].center()[0]<min)|(clouds[i].center()[0]>max)) {
                    finalClouds.push_back(clouds[i].cloud);
                }
            }
            break;
        case 3://Center_y
            if (operation==0) {
                if(clouds[i].center()[1]>=min&&clouds[i].center()[1]<=max) {
                    finalClouds.push_back(clouds[i].cloud);
                }
            } else {
                if((clouds[i].center()[1])<min|(clouds[i].center()[1]>max)) {
                    finalClouds.push_back(clouds[i].cloud);
                }
            }
            break;
        case 4://Center_z
            if (operation==0) {
                if(clouds[i].center()[2]>=min&&clouds[i].center()[2]<=max) {
                    finalClouds.push_back(clouds[i].cloud);
                }
            } else {
                if((clouds[i].center()[2]<min)|(clouds[i].center()[2]>max)) {
                    finalClouds.push_back(clouds[i].cloud);
                }
            }
            break;
        }
    }
    return finalClouds;
}
