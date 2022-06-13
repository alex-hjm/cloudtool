#ifndef CLOUD_H
#define CLOUD_H
#include <QDebug>
#include <QFileInfo>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

typedef pcl::Normal Normal;
typedef pcl::PointNormal PointN;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZRGBNormal PointXYZRGBN;
typedef pcl::PointXYZRGBA PointXYZRGBA;

typedef pcl::PointCloud<Normal> CloudNormal;
typedef pcl::PointCloud<PointN> CloudN;
typedef pcl::PointCloud<PointXYZ> CloudXYZ;
typedef pcl::PointCloud<PointXYZRGBN> CloudXYZRGBN;
typedef pcl::PointCloud<PointXYZRGBA> CloudXYZRGBA;

typedef pcl::PointCloud<pcl::PFHSignature125> PFHFeature;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFHFeature;
typedef pcl::PointCloud<pcl::VFHSignature308> VFHFeature;
typedef pcl::PointCloud<pcl::SHOT352> SHOTFeature;

struct Box{
    double width;
    double height;
    double depth;
    Eigen::Affine3f affine;
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
};

class Cloud:public pcl::PointCloud<PointXYZRGBN>
{
public:
    Cloud():fpfh_feature(new FPFHFeature),shot_feature(new SHOTFeature){}

    Box box;
    PointXYZRGBN min,max;
    FPFHFeature::Ptr fpfh_feature;
    SHOTFeature::Ptr shot_feature;

    std::string id="";
    std::string type="";
    std::string path="";
    std::string box_id="";
    std::string normals_id="";

    int point_size=1;
    float opacity=1.0f;
    float resolution=0.0f;
    bool has_normals=false;

    using Ptr = std::shared_ptr<Cloud>;
    using ConstPtr = std::shared_ptr<const Cloud>;
    inline float volume() const { return box.depth*box.height*box.width; }
    inline Eigen::Vector3f center() const{return box.translation;}
    inline void setId(const std::string& _id) { id=_id; box_id=id+"-box";normals_id=id+"-normals";}
    inline void setPath(const std::string& _path) { path=_path;}
    inline void setInfo(const QFileInfo& _info) {setId(_info.baseName().toStdString());setPath(_info.path().toStdString());}
    inline void setFeature(const FPFHFeature::Ptr& _fpfh) {*fpfh_feature=*_fpfh;}
    inline void setFeature(const SHOTFeature::Ptr& _shot) {*shot_feature=*_shot;}
    inline void rename(const std::string& name) { setId(name);}
    inline void prefix(const std::string& pre)  { setId(pre+id);}
    inline void suffix(const std::string& suf)  { setId(id+suf);}
    inline void copyInfo(const Cloud::Ptr &cloud) {setId(cloud->id);setPath(cloud->path);}
    inline void update() {computerBox();computerType();computeResolution();}
    inline Ptr makeShared() const { return Ptr (new Cloud(*this)); }
    void computerBox()
    {
        pcl::getMinMax3D(*this, min, max);
        Eigen::Vector3f center= 0.5f*(min.getVector3fMap() + max.getVector3fMap());
        Eigen::Vector3f whd=max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f affine=pcl::getTransformation(center[0],center[1],center[2],0,0,0);
        box={whd(0),whd(1),whd(2),affine,center,Eigen::Quaternionf(Eigen::Matrix3f::Identity())};
    }
    void computerType()
    {
        if(points[0].normal_x==0.0f && points[0].normal_y==0.0f&& points[0].normal_z==0.0f) {
            has_normals=false;
            if(points[0].r==0 && points[0].g==0&& points[0].b==0)
                type="XYZ";
            else
                type="XYZRGB";
        } else {
            has_normals=true;
            if(points[0].r==0 && points[0].g==0 && points[0].b==0)
                type="XYZNormal";
            else
                type="XYZRGBNormal";
        }
    }
    void computeResolution()
    {
        int n_points = 0;
        int nres;
        int size_cloud=size();
        int step=size_cloud/20;
        std::vector<int> indices (2);
        std::vector<float> sqr_distances (2);
        pcl::search::KdTree<PointXYZRGBN> tree;
        tree.setInputCloud (this->makeShared());
        for (std::size_t i = 0; i <size_cloud;i+=step) {
            if (! std::isfinite (points[i].x)) {
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
