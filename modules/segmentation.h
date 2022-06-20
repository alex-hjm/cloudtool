/**
 * @file segmentation.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#ifndef CT_MODULES_SEGMENTATION_H
#define CT_MODULES_SEGMENTATION_H

#include <QObject>

#include "base/cloud.h"
#include "base/exports.h"

#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

namespace ct
{
    typedef pcl::PointNormal                                    PointN;
    typedef pcl::PointIndices                                   PointIndices;
    typedef pcl::PointIndicesPtr                                PointIndicesPtr;
    typedef pcl::ModelCoefficients                              ModelCoefficients;
    typedef std::vector<PointIndices>                           IndicesClusters;
    typedef std::shared_ptr<std::vector<pcl::PointIndices>>     IndicesClustersPtr;
    typedef std::function<bool(const PointXYZRGBN&, const PointXYZRGBN&, float)> ConditionFunction;

    class CT_EXPORT Segmentation : public QObject
    {
        Q_OBJECT

    public:
        explicit Segmentation(QObject* parent = nullptr)
            : QObject(parent), cloud_(nullptr), negative_(false)
        {}

        /**
         * @brief 设置输入点云
         */
        void setInputCloud(const Cloud::Ptr& cloud) { cloud_ = cloud; }

        /**
         * @brief 设置是应用点过滤的常规条件，还是应用倒置条件
         */
        void setNegative(bool negative) { negative_ = negative; }

    private:
        Cloud::Ptr cloud_;
        bool negative_;

        std::vector<Cloud::Ptr> getClusters(const IndicesClustersPtr& clusters);

        std::vector<Cloud::Ptr> getClusters(const PointIndicesPtr& clusters);

    signals:

        void segmentationResult(const QString& id, const std::vector<Cloud::Ptr>& cloud, float time, const ModelCoefficients::Ptr& cofe = nullptr);

    public slots:

        /**
         * @brief 基于欧几里得距离和用户定义的聚类执行分割
         * @param func 设置需要满足的条件，以便将相邻点视为同一集群的一部分
         * @param cluster_tolerance 为新的集群候选者设置空间容差
         * @param min_cluster_size 设置集群需要包含的最小点数才能被视为有效
         * @param max_cluster_size 设置集群需要包含的最大点数才能被视为有效
         */
        void ConditionalEuclideanClustering(ConditionFunction func, float cluster_tolerance, int min_cluster_size, int max_cluster_size);

        void DonSegmentation(double mean_radius, double scale1, double scale2, double threshold,
                             double segradius, int minClusterSize, int maxClusterSize);


        /**
         * @brief 表示用于欧几里得意义上的聚类提取的分割类
         * @param tolerance 将空间聚类容差设置为 L2 欧几里得空间中的度量
         * @param min_cluster_size 设置集群需要包含的最小点数才能被视为有效
         * @param max_cluster_size 设置集群需要包含的最大点数才能被视为有效
         */
        void EuclideanClusterExtraction(double tolerance, int min_cluster_size, int max_cluster_size);

        /**
         * @brief 使用一组表示平面模型的点索引，并与给定的高度一起生成 3D 多边形棱柱
         * @param hull 提供指向输入平面船体数据集的指针
         * @param height_min height_max 设置高度限制
         * @param vpx vpy vpz 设置视点
         */
        void ExtractPolygonalPrismData(const Cloud::Ptr& hull, double height_min, double height_max, float vpx, float vpy, float vpz);

        /** TODO
         * @brief
         *
         * @param sigma
         * @param radius
         * @param weight
         * @param neighbour_number
         */
        void MinCutSegmentation(double sigma, double radius, double weight, int neighbour_number);

        /** TODO
         * @brief
         *
         * @param max_window_size
         * @param slope
         * @param max_distance
         * @param initial_distance
         * @param cell_size
         * @param base
         * @param negative
         */
        void MorphologicalFilter(int max_window_size, float slope, float max_distance, float initial_distance,
                                 float cell_size, float base, bool negative);


        /**
         * @brief 实现用于分割的众所周知的区域增长算法
         * @param min_cluster_size 设置集群需要包含的最小点数才能被视为有效
         * @param max_cluster_size 设置集群需要包含的最大点数才能被视为有效
         * @param smooth_mode 允许打开/关闭平滑约束
         * @param curvature_test 允许打开/关闭曲率测试
         * @param residual_test 允许打开/关闭剩余测试
         * @param smoothness_threshold 允许设置用于测试点的平滑度阈值
         * @param residual_threshold 允许设置用于测试点的残差阈值
         * @param curvature_threshold 允许设置用于测试点的曲率阈值
         * @param neighbours 允许设置邻居的数量
         */
        void RegionGrowing(int min_cluster_size, int max_cluster_size, bool smooth_mode, bool curvature_test, bool residual_test,
                           float smoothness_threshold, float residual_threshold, float curvature_threshold, int neighbours);

        /**
         * @brief 实现用于基于点颜色进行分割的众所周知的区域增长算法
         * @param min_cluster_size 设置集群需要包含的最小点数才能被视为有效
         * @param max_cluster_size 设置集群需要包含的最大点数才能被视为有效
         * @param smooth_mode 允许打开/关闭平滑约束
         * @param curvature_test 允许打开/关闭曲率测试
         * @param residual_test 允许打开/关闭剩余测试
         * @param smoothness_threshold 允许设置用于测试点的平滑度阈值
         * @param residual_threshold 允许设置用于测试点的残差阈值
         * @param curvature_threshold 允许设置用于测试点的曲率阈值
         * @param neighbours 允许设置邻居的数量
         * @param pt_thresh 指定点之间颜色测试的阈值
         * @param re_thresh 指定区域之间颜色测试的阈值
         * @param dis_thresh 允许设置距离阈值
         * @param nghbr_number 允许设置用于查找相邻段的邻居数
         */
        void RegionGrowingRGB(int min_cluster_size, int max_cluster_size, bool smooth_mode, bool curvature_test,
                              bool residual_test, float smoothness_threshold, float residual_threshold, float curvature_threshold,
                              int neighbours, float pt_thresh, float re_thresh, float dis_thresh, int nghbr_number);

        /**
         * @brief 表示 Sample Consensus 方法和模型的 Nodelet
         * 分割类，从某种意义上说，它只是为基于 SAC 的通用分割 创建了一个 Nodelet
         * 包装器
         * @param model 要使用的模型类型（用户给定参数）
         * @param method 要使用的样本共识方法的类型（用户给定的参数）
         * @param threshold 到模型阈值的距离（用户给定参数）
         * @param max_iterations 设置放弃前的最大迭代次数
         * @param probability 设置至少选择一个没有异常值的样本的概率
         * @param optimize 如果需要进行系数细化，则设置为 true
         * @param min_radius max_radius 设置模型的最小和最大允许半径限制（适用于估计半径的模型）
         */
        void SACSegmentation(int model, int method, double threshold, int max_iterations, double probability, bool optimize,
                             double min_radius, double max_radius);

        /**
         * @brief 表示 Sample Consensus 方法和模型的 Nodelet
         * 分割类，从某种意义上说，它只是为基于 SAC 的通用分割 创建了一个 Nodelet
         * 包装器
         * @param model 要使用的模型类型（用户给定参数）
         * @param method 要使用的样本共识方法的类型（用户给定的参数）
         * @param threshold 到模型阈值的距离（用户给定参数）
         * @param max_iterations 设置放弃前的最大迭代次数
         * @param probability 设置至少选择一个没有异常值的样本的概率
         * @param optimize 如果需要进行系数细化，则设置为 true
         * @param min_radius max_radius 设置模型的最小和最大允许半径限制（适用于估计半径的模型）
         * @param distance_weight 设置相对权重（介于 0 和 1 之间）以给出点法线和平面法线之间的角距离（0 到 pi/2）
         * @param d 设置期望平面模型与原点的距离
         */
        void SACSegmentationFromNormals(int model, int method, double threshold, int max_iterations, double probability,
                                        bool optimize, double min_radius, double max_radius, double distance_weight, double d);

        /**
         * @brief 种子色调分割
         * @param tolerance 将空间聚类容差设置为 L2 欧几里得空间中的度量
         * @param delta_hue 设置色相的容差
         */
        void SeededHueSegmentation(double tolerance, float delta_hue);

        /**
         * @brief
         * 获得两个空间对齐的点云之间的差异，并返回最大给定距离阈值的它们之间的差异
         * @param tar_cloud 提供指向目标数据集的指针，将与 setInputCloud
         * 中给出的输入云进行比较
         * @param sqr_threshold 设置两个输入数据集中对应点之间的最大距离容差（平方）
         */
        void SegmentDifferences(const Cloud::Ptr& tar_cloud, double sqr_threshold);

        /**
         * @brief 实现基于体素结构、法线和 rgb 值的超体素算法。
         * @param voxel_resolution 设置八叉树体素的分辨率
         * @param seed_resolution 设置八叉树种子体素的分辨率
         * @param color_importance 设置颜色对超体素的重要性
         * @param spatial_importance 设置空间距离对超体素的重要性
         * @param normal_importance 设置超体素标量正态积的重要性
         * @param camera_transform 设置是否使用单相机变换
         */
        void SupervoxelClustering(float voxel_resolution, float seed_resolution, float color_importance, float spatial_importance,
                                  float normal_importance, bool camera_transform);

    };
}  // namespace pca

#endif  // PCA_SEGMENTATION_H
