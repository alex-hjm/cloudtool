#ifndef CLOUD_H
#define CLOUD_H
#include <QDebug>
#include <QFileInfo>
#include <pcl/search/kdtree.h>
#include "customtype.h"

class Cloud
{
public:
    Cloud():cloud(new CloudXYZRGBN),keypoints(new CloudXYZRGBN),
        fpfhFeature(new FPFHFeature),shotFeature(new SHOTFeature),
        correspondences(new Correspondences),referenceframe(new ReferenceFrame)
    {}

    Cloud(CloudXYZRGBN::Ptr &cloudin,const QFileInfo &info):
        cloud(cloudin),fileInfo(info),keypoints(new CloudXYZRGBN),
        fpfhFeature(new FPFHFeature),shotFeature(new SHOTFeature),
        correspondences(new Correspondences),referenceframe(new ReferenceFrame)
    {
        id=fileInfo.baseName().toStdString();
        boxid=id+"box";
        normalsid=id+"normals";
        keyid=id+"key";
        this->computeBox();
        this->computeType();
        this->computeResolution();
    }

    void
    update()
    {
        this->computeBox();
        this->computeType();
        this->computeResolution();
    }

    CloudXYZRGBN::Ptr cloud;
    CloudXYZRGBN::Ptr keypoints;
    FPFHFeature::Ptr fpfhFeature;
    SHOTFeature::Ptr shotFeature;
    CorrespondencesPtr correspondences;
    ReferenceFrame::Ptr referenceframe;

    PointXYZRGBN min;
    PointXYZRGBN max;
    QFileInfo fileInfo;
    QString type="";
    BoundingBox box;
    int pointSize=1.0;
    float opacity=1.0;
    float resolution=0;
    bool hasNormals=false;
    std::string id="";
    std::string boxid="";
    std::string normalsid="";
    std::string keyid="";

    inline bool hasKeypts () const { return !keypoints->empty(); }
    inline bool hasFPFH () const { return !fpfhFeature->empty(); }
    inline bool hasSHOT () const { return !shotFeature->empty(); }
    inline bool hasCorrs () const { if(correspondences->size()==0) return false; else return true;}
    inline bool hasLRF () const { return !referenceframe->empty(); }
    inline bool empty () const { return cloud->empty (); }

    inline std::size_t size () const { return cloud->size (); }
    inline QString path() const { return fileInfo.path(); }
    inline float fileSize() const { return fileInfo.size()/1024;} //KB
    inline float volume() const { return box.depth*box.height*box.width; }
    inline Eigen::Vector3f center() const{return box.translation;}

    inline void rename(const std::string & name) { id=name; boxid=id+"box";normalsid=id+"normals",keyid=id+"key";
                                                   fileInfo.setFile(fileInfo.path()+"/"+id.c_str());}
    inline void append(const std::string & name) { id=id+name; boxid=id+"box";normalsid=id+"normals",keyid=id+"key";
                                                   fileInfo.setFile(fileInfo.path()+"/"+id.c_str());}
    inline void prefix(const std::string & name) { id=name+id ;boxid=id+"box";normalsid=id+"normals",keyid=id+"key";
                                                   fileInfo.setFile(fileInfo.path()+"/"+id.c_str());}
protected:
    void
    computeBox()
    {
        pcl::getMinMax3D(*cloud, min, max);
        Eigen::Vector3f cloud_center= 0.5f*(min.getVector3fMap() + max.getVector3fMap());
        Eigen::Vector3f whd;
        whd = max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f affine=Eigen::Affine3f::Identity();
        affine=pcl::getTransformation(cloud_center[0],cloud_center[1],cloud_center[2],0,0,0);
        box={whd(0),whd(1),whd(2),affine,cloud_center,Eigen::Quaternionf(Eigen::Matrix3f::Identity())};
    }

    void
    computeType()
    {
        if(cloud->points[0].normal_x==0.0f && cloud->points[0].normal_y==0.0f
                && cloud->points[0].normal_z==0.0f) {
            hasNormals=false;
            if(cloud->points[0].r==0 && cloud->points[0].g==0&& cloud->points[0].b==0)
                type="XYZ";
            else
                type="XYZRGB";
        }
        else {
            hasNormals=true;
            if(cloud->points[0].r==0 && cloud->points[0].g==0 && cloud->points[0].b==0)
                type="XYZNormal";
            else
                type="XYZRGBNormal";
        }
    }

    void
    computeResolution()
    {
        int n_points = 0;
        int nres;
        std::vector<int> indices (2);
        std::vector<float> sqr_distances (2);
        pcl::search::KdTree<PointXYZRGBN> tree;
        tree.setInputCloud (cloud);
        for (std::size_t i = 0; i < cloud->size (); ++i) {
            if (! std::isfinite ((*cloud)[i].x)) {
                continue;
            }
            nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
            if (nres == 2) {
                resolution += sqrt (sqr_distances[1]);
                ++n_points;
            }
        }
        if (n_points != 0) {
            resolution /= n_points;
        }
    }
};


#endif // CLOUD_H
