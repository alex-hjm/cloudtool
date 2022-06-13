#ifndef Common_H
#define Common_H
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "src/common/cloud.h"
#include "src/common/customtype.h"
#include "src/common/tool.h"

class Common
{
public:
    Common();
    //color
    CloudXYZRGBN::Ptr setColor(const CloudXYZRGBN::Ptr &cloud,int r,int g,int b);
    CloudXYZRGBN::Ptr setColor(const CloudXYZRGBN::Ptr &cloud,string aixs);

    //scale
    CloudXYZRGBN::Ptr ScaleCloud(const CloudXYZRGBN::Ptr &cloud,const Eigen::Vector3f &center,
                                 double scaleX,double scaleY,double scaleZ,const bool &origin);
    //selection
    std::vector<CloudXYZRGBN::Ptr> SelectCloud(const std::vector<Cloud> &clouds,int feature,int operation,float min,float max);

private:
   Tool tool;
};

#endif // Common_H
