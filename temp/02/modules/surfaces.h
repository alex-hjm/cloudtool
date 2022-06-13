#ifndef SURFACES_H
#define SURFACES_H

#include <pcl/surface/grid_projection.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>

#include "src/common/customtype.h"

class Surfaces
{
public:
    Surfaces();

    float tocTime;
    PolygonMesh::Ptr GreedyProjectionTriangulation(const CloudXYZRGBN::Ptr &cloud, double mu,int nnn,double radius,int min,int max,int surface);
    PolygonMesh::Ptr GridProjection(const CloudXYZRGBN::Ptr &cloud, double resolution,int k,int max_binary_search_level);
    PolygonMesh::Ptr Poisson(const CloudXYZRGBN::Ptr &cloud,int depth, int min_depth,float point_weight,float scale,float samples_per_node);

private:
    pcl::console::TicToc time;
};

#endif // SURFACE_H
