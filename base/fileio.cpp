/**
 * @file fileio.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#include "base/fileio.h"

#include <pcl/filters/filter.h>
#include <pcl/io/ifs_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace ct
{
  void FileIO::loadPointCloud(const QString& filename)
  {
    TicToc time;
    time.tic();
    Cloud::Ptr cloud(new Cloud);
    QFileInfo fileinfo(filename);
    int result = -1;
    if (fileinfo.suffix() == "pcd")
      result = pcl::io::loadPCDFile(filename.toLocal8Bit().toStdString(), *cloud);
    else if (fileinfo.suffix() == "ply")
      result = pcl::io::loadPLYFile(filename.toLocal8Bit().toStdString(), *cloud);
    else if (fileinfo.suffix() == "obj")
      result = pcl::io::loadOBJFile(filename.toLocal8Bit().toStdString(), *cloud);
    else if (fileinfo.suffix() == "ifs")
      result = pcl::io::loadIFSFile(filename.toLocal8Bit().toStdString(), *cloud);
    else {
      emit loadCloudResult(false, cloud, time.toc());
      return;
    }
    if (result == -1) {
      emit loadCloudResult(false, cloud, time.toc());
      return;
    }
    // remove NaN
    cloud->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    // cloud init
    cloud->setId(fileinfo.baseName());
    cloud->setInfo(fileinfo);
    cloud->update();
    emit loadCloudResult(true, cloud, time.toc());
  }

  void FileIO::savePointCloud(const Cloud::Ptr& cloud, const QString& filename, bool isBinary)
  {
    TicToc time;
    time.tic();
    QFileInfo fileinfo(filename);
    int result = -1;
    if (fileinfo.suffix() == "pcd")
      result = pcl::io::savePCDFile(filename.toLocal8Bit().toStdString(), *cloud, isBinary);
    else if (fileinfo.suffix() == "ply")
      result = pcl::io::savePLYFile(filename.toLocal8Bit().toStdString(), *cloud, isBinary);
    else
      result = pcl::io::savePLYFile(QString(filename + ".ply").toLocal8Bit().toStdString(), *cloud, isBinary);
    if (result == -1)
      emit saveCloudResult(false, filename, time.toc());
    else
      emit saveCloudResult(true, filename, time.toc());
  }
}