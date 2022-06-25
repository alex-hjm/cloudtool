/**
 * @file features.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#ifndef CT_MODULES_FEATURES_H
#define CT_MODULES_FEATURES_H

#include "base/exports.h"
#include "base/cloud.h"

#include <QObject>

#include <pcl/point_types.h>

namespace ct
{

    typedef pcl::PointCloud<pcl::ShapeContext1980>          SC3DFeature;
    typedef pcl::PointCloud<pcl::Histogram<90>>             CRHFeature;
    typedef pcl::PointCloud<pcl::VFHSignature308>           VFHFeature;
    typedef pcl::PointCloud<pcl::ESFSignature640>           ESFFeature;
    typedef pcl::PointCloud<pcl::FPFHSignature33>           FPFHFeature;
    typedef pcl::PointCloud<pcl::GASDSignature512>          GASDFeature;
    typedef pcl::PointCloud<pcl::GASDSignature984>          GASDCFeature;
    typedef pcl::PointCloud<pcl::GRSDSignature21>           GRSDFeature;
    typedef pcl::PointCloud<pcl::PFHSignature125>           PFHFeature;
    typedef pcl::PointCloud<pcl::PrincipalRadiiRSD>         RSDFeature;
    typedef pcl::PointCloud<pcl::SHOT352>                   SHOTFeature;
    typedef pcl::PointCloud<pcl::SHOT1344>                  SHOTCFeature;
    typedef pcl::PointCloud<pcl::UniqueShapeContext1960>    USCFeature;
    typedef pcl::PointCloud<pcl::ReferenceFrame>            ReferenceFrame;

    struct FeatureType
    {
        using Ptr = std::shared_ptr<FeatureType>;
        using ConstPtr = std::shared_ptr<const FeatureType>;

        PFHFeature::Ptr     pfh;
        FPFHFeature::Ptr    fpfh;
        VFHFeature::Ptr     vfh;
        ESFFeature::Ptr     esf;
        SC3DFeature::Ptr    sc3d;
        GASDFeature::Ptr    gasd;
        CRHFeature::Ptr     crh;
        GRSDFeature::Ptr    grsd;
        GASDCFeature::Ptr   gasdc;
        RSDFeature::Ptr     rsd;
        SHOTFeature::Ptr    shot;
        SHOTCFeature::Ptr   shotc;
        USCFeature::Ptr     usc;
    };

    class CT_EXPORT Features : public QObject
    {
        Q_OBJECT

    public:
        explicit Features(QObject* parent = nullptr)
            : QObject(parent), cloud_(nullptr), surface_(nullptr), k_(0), radius_(0)
        {}

        void setInputCloud(const Cloud::Ptr& cloud) { cloud_ = cloud; }

        /**
         * @brief 提供指向数据集的指针以添加其他信息以估计输入数据集中每个点的特征
         */
        void setSearchSurface(const Cloud::Ptr& surface) { surface_ = surface; }

        /**
         * @brief 设置用于特征估计的 k 最近邻的数量
         */
        void setKSearch(int k) { k_ = k; }

        /**
         * @brief 设置用于确定用于特征估计的最近邻的球体半径
         */
        void setRadiusSearch(double radius) { radius_ = radius; }

    private:
        Cloud::Ptr cloud_;
        Cloud::Ptr surface_;
        int k_;
        double radius_;

    signals:

        /**
         * @brief 估计法线结果
         */
        void normalsResult(const Cloud::Ptr& cloud, float time);

        /**
         * @brief 边界估计结果
         */
        void boundaryResult(const Cloud::Ptr& cloud, float time);

        /**
         * @brief 法线差异结果
         */
        void donResult(const Cloud::Ptr& cloud, float time);

        /**
         * @brief 本地参考帧估计结果
         */
        void lrfResult(const QString& id, const ReferenceFrame::Ptr& cloud, float time);

        /**
         * @brief 特征估计结果
         */
        void featureResult(const QString& id, const FeatureType::Ptr& feature, float time);

    public slots:

        /**
         * @brief 计算AABB包围盒
         */
        static Box boundingBoxAABB(const Cloud::Ptr&);

        /**
         * @brief 计算OBB包围盒
         */
        static Box boundingBoxOBB(const Cloud::Ptr&);

        /**
         * @brief 计算自定义包围盒
         */
        static Box boundingBoxAdjust(const Cloud::Ptr&, const Eigen::Affine3f& t);

        /**
         * @brief 估计每个 3D 点的局部表面特性（表面法线和曲率）
         * @param vpx vpy vpz 设置视点
         */
        void NormalEstimation(float vpx, float vpy, float vpz);

        /**
         * @brief 点云数据的法线差 (DoN) 尺度过滤器实现
         * @param small_normal 为 DoN 运算符设置使用较小搜索半径（比例）计算的法线
         * @param large_normal 为 DoN 运算符设置使用较大搜索半径（比例）计算的法线
         */
        void DifferenceOfNormalsEstimation(const Cloud::Ptr& small_normal, const Cloud::Ptr& large_normal);

        /**
         * @brief 使用角度标准估计一组点是否位于表面边界上
         * @param angle 设置将点标记为边界或规则的决策边界（角度阈值）
         */
        void BoundaryEstimation(float angle);

        /**
         * @brief 估计包含点和法线的给定点云数据集的点特征直方图 (PFH) 描述符
         */
        void PFHEstimation();

        /**
         * @brief 估计包含点和法线的给定点云数据集的快速点特征直方图 (FPFH) 描述符
         */
        void FPFHEstimation();

        /**
         * @brief VFHEstimation
         * @param dir 设置视图方向
         */
        void VFHEstimation(const Eigen::Vector3f& dir);

        /**
         * @brief 估计包含点的给定点云数据集的形状函数描述符集
         */
        void ESFEstimation();

        /**
         * @brief 在给定 XYZ 数据的情况下，估计给定点云数据集的全局对齐空间分布 (GASD) 描述符
         * @param dir 设置视图方向
         * @param shgs 设置形状半网格大小
         * @param shs 设置形状直方图大小
         * @param interp 设置形状直方图插值方法 0-NONE 1-TRILINEAR 2-QUADRILINEAR
         */
        void GASDEstimation(const Eigen::Vector3f& dir, int shgs, int shs, int interp);

        /**
         * @brief 在给定 XYZ 和 RGB 数据的情况下，估计给定点云数据集的全局对齐空间分布
         * (GASD) 描述符
         * @param dir 设置视图方向
         * @param shgs 设置形状半网格大小
         * @param shs 设置形状直方图大小
         * @param interp 设置形状直方图插值方法 0-NONE 1-TRILINEAR 2-QUADRILINEAR
         * @param chgs 设置颜色半网格大小
         * @param chs 设置颜色直方图大小
         * @param cinterp 设置颜色直方图插值方法 0-NONE 1-TRILINEAR 2-QUADRILINEAR
         */
        void GASDColorEstimation(const Eigen::Vector3f& dir, int shgs, int shs, int interp,
                                 int chgs, int chs, int cinterp);


        /**
         * @brief
         * 估计包含点和法线的给定点云数据集的基于半径的曲面描述符（局部曲面曲线的最小和最大半径）
         * @param nr_subdiv 为考虑的距离间隔设置细分数
         * @param plane_radius 设置最大半径，在该半径之上，一切都可以被认为是平面的
         */
        void RSDEstimation(int nr_subdiv, double plane_radius);

        /**
         * @brief 估计包含点和法线的给定点云数据集的基于全局半径的表面描述符 (GRSD)
         */
        void GRSDEstimation();

        /**
         * @brief 估计包含 XYZ 数据和法线的给定点云数据集的相机滚动直方图 (CRH) 描述符
         * @param dir 设置视图方向
         */
        void CRHEstimation(const Eigen::Vector3f& dir);

        /**
         * @brief 估计包含 XYZ 数据和法线的给定点云数据集的聚类视点特征直方图 (CVFH) 描述符
         * @param dir 设置视图方向
         * @param radius_normals 设置用于计算法线的半径
         * @param d1 设置聚类最大值 （要添加到集群的点之间的欧几里得距离）
         * @param d2 设置最大值 （两点之间法线的偏差，因此它们可以聚集在一起）
         * @param d3 设置去除法线的曲率阈值
         * @param min 设置要考虑的集群的最小点数
         * @param normalize 设置是否应标准化 CVFH 签名
         */
        void CVFHEstimation(const Eigen::Vector3f& dir, float radius_normals,
                            float d1, float d2, float d3, int min, bool normalize);

        /**
         * @brief 实现 3D 形状上下文描述符
         * @param min_radius 搜索球体的最小半径值 (rmin)
         * @param radius 此半径用于计算局部点密度密度 = 此半径内的点数
         */
        void ShapeContext3DEstimation(double min_radius, double radius);

        /**
         * @brief 估计包含点和法线的给定点云数据集的方向直方图 (SHOT) 描述符的签名
         * @param lrf 提供指向包含 XYZ 数据集的本地参考帧的输入数据集的指针
         * @param radius 如果用户未设置帧，则设置用于局部参考帧估计的半径
         */
        void SHOTEstimation(const ReferenceFrame::Ptr& lrf, float radius);

        /**
         * @brief 估计包含点、法线和颜色的给定点云数据集的方向直方图 (SHOT)
         * 描述符的签名
         * @param lrf 提供指向包含 XYZ 数据集的本地参考帧的输入数据集的指针
         * @param radius 如果用户未设置帧，则设置用于局部参考帧估计的半径
         */
        void SHOTColorEstimation(const ReferenceFrame::Ptr& lrf, float radius);

        /**
         * @brief 唯一形状上下文描述符
         * @param lrf 提供指向包含 XYZ 数据集的本地参考帧的输入数据集的指针
         * @param min_radius 原始论文中搜索球体的最小半径值 (rmin)
         * @param pt_radius 此半径用于计算局部点密度密度 = 此半径内的点数
         * @param loc_radius 设置本地射频半径值
         */
        void UniqueShapeContext(const ReferenceFrame::Ptr& lrf, double min_radius,
                                double pt_radius, double loc_radius);
                                
        /**
         * @brief 局部参考帧估计实现边界感知可重复方向算法
         * @param radius 设置用于估计给定点的板参考框架的 x 轴和 y 轴的点的最大距离。
         * @param find_holes
         * 设置是否在参考框架的估计中搜索和考虑每个点的支撑边缘中的孔
         * @param margin_thresh
         * 设置搜索半径（或切线半径，如果设置）的百分比，在该百分比之后点被视为支撑边距的一部分
         * @param size 设置用于搜索缺失区域的分割数
         * @param prob_thresh
         * 给定支撑边缘中孔的角度宽度，设置该角度必须覆盖的圆周的最小百分比，
         *                    以被认为是支撑中的缺失区域，因此用于参考框架的估计
         * @param steep_thresh
         * 参考算法找到的最佳点的法线，设置位于孔边界上的点的法线必须具有的最小陡度，
         *                     以便在计算参考框架时考虑
         */
        void BOARDLocalReferenceFrameEstimation(float radius, bool find_holes, float margin_thresh, int size,
                                                float prob_thresh, float steep_thresh);

        /**
         * @brief 为本地参考帧估计实现快速本地参考帧算法
         * @param radius 设置用于估计给定点的 FLARE 参考框架的 x_axis 的点的最大距离
         * @param margin_thresh
         * 设置搜索切线半径的百分比，在该百分比之后点被视为支撑的一部分
         * @param min_neighbors_for_normal_axis 设置计算 Z 轴所需的最小邻居数
         * @param min_neighbors_for_tangent_axis 提供指向用于估计 X 轴的数据集的指针
         */
        void FLARELocalReferenceFrameEstimation(float radius, float margin_thresh, int min_neighbors_for_normal_axis,
                                                int min_neighbors_for_tangent_axis);

        /**
         * @brief 估计在（SHOT）描述符的计算中使用的局部参考帧
         */
        void SHOTLocalReferenceFrameEstimation();
    };
} // namespace ct

#endif // CT_MODULES_FEATURES_H
