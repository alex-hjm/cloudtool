#ifndef FILEIOMODULE_H
#define FILEIOMODULE_H
#include <QTextCodec>
#include <QFileInfo>
#include <pcl/io/auto_io.h>
#include <pcl/io/boost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ifs_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/filter.h>
#include <pcl/console/time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "src/common/customtype.h"

class FileIO
{
public:
    FileIO();

    float tocTime;
    bool loadPointCloud(CloudXYZRGBN::Ptr& cloud,const QFileInfo& fileInfo);
    bool savePointCloud(CloudXYZRGBN::Ptr& cloud,const QFileInfo& fileInfo,bool isBinary);
    void savePointCloud(CloudXYZRGBN::Ptr& cloud,const QFileInfo& fileInfo);
    bool loadImage(cv::Mat& image,QFileInfo& fileInfo);
    bool saveImage(cv::Mat& image,QFileInfo& fileInfo);
private:
    pcl::console::TicToc time;

};

#endif // FILEIOMODULE_H
