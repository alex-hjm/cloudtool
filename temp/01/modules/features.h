#ifndef FEATURES_H
#define FEATURES_H
#include <QObject>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/shot_omp.h>

#include <pcl/features/board.h>
#include <pcl/features/flare.h>
#include <pcl/features/shot_lrf_omp.h>
#ifdef _WIN32
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/impl/pfh.hpp>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/features/impl/fpfh_omp.hpp>
#include <pcl/features/impl/vfh.hpp>
#include <pcl/features/impl/shot.hpp>
#include <pcl/features/impl/shot_lrf.hpp>
#include <pcl/features/impl/shot_lrf_omp.hpp>
#include <pcl/features/impl/shot_omp.hpp>
#include <pcl/features/impl/board.hpp>
#include <pcl/features/impl/flare.hpp>
#endif
#include "common/cloud.h"

class Features:public QObject
{
    Q_OBJECT
public:
   explicit Features(QObject *parent = nullptr);
signals:
    void result(const Cloud::Ptr &cloud,float time);
public slots:
    //normals
    void normalEstimation(const Cloud::Ptr& cloud,int K,double R,const Eigen::Vector3f &viewpoint);
    //boundingbox
    static Box boundingBoxAABB(const Cloud::Ptr&);
    static Box boundingBoxOBB(const Cloud::Ptr&);
    static Box boundingBoxAdjust(const Cloud::Ptr&,const Eigen::Affine3f &t);
    //feature
    static PFHFeature::Ptr PFHEstimation(const Cloud::Ptr&cloud,int k,double radius);
    static FPFHFeature::Ptr FPFHEstimation(const Cloud::Ptr&cloud,int k,double radius);
    static VFHFeature::Ptr VFHEstimation(const Cloud::Ptr&cloud,int k,double radius);
    static SHOTFeature::Ptr SHOTEstimation(const Cloud::Ptr&cloud,int k,double radius);



};

#endif // FEATURES_H
