#ifndef CUSTOMTYPE_H
#define CUSTOMTYPE_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/geometry/planar_polygon.h>
using namespace  std;

typedef pcl::Normal Normal;
typedef pcl::PointNormal PointN;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointXYZL PointXYZL;
typedef pcl::PointXYZRGB PointXYZRGB;
typedef pcl::PointXYZRGBL PointXYZRGBL;
typedef pcl::PointXYZRGBA PointXYZRGBA;
typedef pcl::PointXYZINormal PointXYZIN;
typedef pcl::PointXYZRGBNormal PointXYZRGBN;

typedef pcl::PointCloud<Normal> CloudNormal;
typedef pcl::PointCloud<PointN> CloudN;
typedef pcl::PointCloud<PointXYZ> CloudXYZ;
typedef pcl::PointCloud<PointXYZI> CloudXYZI;
typedef pcl::PointCloud<PointXYZL> CloudXYZL;
typedef pcl::PointCloud<PointXYZRGB> CloudXYZRGB;
typedef pcl::PointCloud<PointXYZRGBL> CloudXYZRGBL;
typedef pcl::PointCloud<PointXYZRGBA> CloudXYZRGBA;
typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
typedef pcl::PointCloud<PointXYZRGBN> CloudXYZRGBN;

typedef pcl::ModelCoefficients ModelCoefs;
typedef pcl::PolygonMesh PolygonMesh ;
typedef pcl::PlanarPolygon<PointXYZRGBN> PolyXYZRGBN;

//feature
typedef pcl::PointCloud<pcl::PFHSignature125> PFHFeature;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFHFeature;
typedef pcl::PointCloud<pcl::VFHSignature308> VFHFeature;
typedef pcl::PointCloud<pcl::SHOT352> SHOTFeature;
typedef pcl::PointCloud<pcl::ReferenceFrame> ReferenceFrame;

typedef pcl::Correspondences Correspondences;
typedef pcl::CorrespondencesPtr CorrespondencesPtr;


enum left_panel
{
    color=0,
    normals,
    boundingbox,
    filter,
    transform,
    treesearch,
    surface,
    features,
    recognition,
    segmentation,
    factory,
    device_azurekinect,
    device_zhisensor
};

enum right_panel
{
    path_plan=0,
    process_plan,
    keypoint,
    registration,
    correspondence,
    calibration,
    range_image
};

enum dialog_panel
{
    coordinate=0,
    scale,
    sampling,
    measure,
    pickpoints,
    segment,
    selection,
    connection
};

enum model_type
{
    Plane=0,
    Sphere,
    Line,
    Cylinder,
    Circle,
    Cone,
    Cube,
    Arrow,
    Text3D,
    Mesh,
    polygon,
    polyline,
};

enum process_type
{
    process_color,
    process_boundingBox,
    process_fiter,
    process_normols,
    process_scale,
    process_transform,
    process_segmentation,
    process_surface,
    process_keypoint,
    process_treesearch,
    process_coordinate,
    process_device
};

struct BoundingBox{
    double width;
    double height;
    double depth;
    Eigen::Affine3f affine;
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
};

struct Model
{
    std::string id="";
    std::string text="";
    model_type type;
    ModelCoefs coefs;
    PolygonMesh::Ptr mesh;
    PolyXYZRGBN polygon;
    CloudXYZRGBN::Ptr points;
    float pointSize=1;
    float fontSize=0;
    float opacity=1;
    float linewidth=1;
    float r=255;
    float g=255;
    float b=255;
    int repersentation=2;
    int shading=0;
    bool display_length=false;
    bool isEmpty=true;
};

#endif // CUSTOMTYPE_H
