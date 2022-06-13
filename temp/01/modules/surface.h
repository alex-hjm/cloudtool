#ifndef SURFACE_H
#define SURFACE_H

#include <QObject>
#include <pcl/console/time.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include "common/cloudview.h"

class Surface:public QObject
{
    Q_OBJECT
public:
    explicit Surface(QObject *parent = nullptr);
signals:
    void result(const PolygonMesh::Ptr &mesh,const std::string &id,float time);
public slots:
    void greedyProjectionTriangulation(const Cloud::Ptr &cloud, double mu,int nnn,double radius,int min,int max,int surface);
    void gridProjection(const Cloud::Ptr &cloud, double resolution,int k,int max_binary_search_level);
    void poisson(const Cloud::Ptr &cloud,int depth, int min_depth,float point_weight,float scale,float samples_per_node);
};

#endif // SURFACE_H
