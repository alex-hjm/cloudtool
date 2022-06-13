#ifndef Common_H
#define Common_H
#include "common/cloud.h"
#include "common/tool.h"

class Common
{

public:
    //color
    static void setColor(const Cloud::Ptr &cloud,int r,int g,int b);
    static void setColor(const Cloud::Ptr &cloud,const std::string &aixs);
    //scale
    static Cloud::Ptr ScaleCloud(const Cloud::Ptr &cloud,double scaleX,double scaleY,double scaleZ,const bool &origin);
};

#endif // Common_H
