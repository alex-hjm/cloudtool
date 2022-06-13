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
};

#endif // Common_H
