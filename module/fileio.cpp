/**
 * @file fileio.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-02
 */
#include "fileio.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>

#include <QFileInfo>

CT_BEGIN_NAMESPACE

void FileIO::loadCloudFile(const QString& file_name) 
{
    int result = -1;
    QFileInfo fileinfo(file_name);
    Cloud::Ptr cloud(new Cloud);
    if (fileinfo.suffix() == "pcd")
        result = pcl::io::loadPCDFile(file_name.toLocal8Bit().toStdString(), *cloud);
    else if (fileinfo.suffix() == "ply")
        result = pcl::io::loadPLYFile(file_name.toLocal8Bit().toStdString(), *cloud);
    if (-1 == result)  {
        loadCloudResult(false, nullptr);
        return;
    }
    cloud->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    cloud->setId(fileinfo.baseName());
    cloud->setPath(fileinfo.absolutePath());
    cloud->updateBBox();
    cloud->updateResolution();
    loadCloudResult(true, cloud);   
}

void FileIO::saveCloudFile(const Cloud::Ptr& cloud, const QString& file_name, bool isBinary)
{
    QFileInfo fileinfo(file_name);
    int result = -1;
    if (fileinfo.suffix() == "pcd")
        result = pcl::io::savePCDFile(file_name.toLocal8Bit().toStdString(), *cloud, isBinary);
    else if (fileinfo.suffix() == "ply")
        result = pcl::io::savePLYFile(file_name.toLocal8Bit().toStdString(), *cloud, isBinary);
    else
        result = pcl::io::savePLYFile(file_name.toLocal8Bit().toStdString() + ".ply", *cloud, isBinary);
    if (-1 == result)  {
        saveCloudResult(false);
        return;
    }
    cloud->setPath(fileinfo.absolutePath());
    saveCloudResult(true);
}

CT_END_NAMESPACE
