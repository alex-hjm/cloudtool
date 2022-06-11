/**
 * @file filters.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#ifndef CT_MODULES_FILTERS_H
#define CT_MODULES_FILTERS_H

#include "base/exports.h"
#include "base/cloud.h"

#include <QObject>

#include <pcl/Vertices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/surface/mls.h>


namespace ct
{
    typedef pcl::ConditionBase<PointXYZRGBN> Condition;
    typedef pcl::PointXYZRGB PointXYZRGB;

    class CT_EXPORT Filters : public QObject
    {
        Q_OBJECT
    public:
        explicit Filters(QObject* parent = nullptr)
            : QObject(parent),
            cloud_(),
            negative_(false),
            keep_organized_(false),
            value_(std::numeric_limits<float>::quiet_NaN())
        {}

        void setInputCloud(const Cloud::Ptr& cloud) { cloud_ = cloud; }

        /**
         * @brief 设置是应用点过滤的常规条件，还是应用倒置条件
         */
        void setNegative(bool negative) { negative_ = negative; }

        /**
         * @brief 设置过滤后的点是否应保留并设置为通过 setUserFilterValue 给出的值
         *        （默认值：NaN）,或者从 PointCloud 中删除，从而可能破坏其组织结构。
         */
        void setKeepOrganized(bool keep_organized)
        {
            keep_organized_ = keep_organized;
        }

        /**
         * @brief 提供一个值，过滤点应设置为而不是删除它们
         */
        void setUserFilterValue(float value) { value_ = value; }

    private:
        Cloud::Ptr cloud_;
        bool negative_;
        bool keep_organized_;
        float value_;

    signals:

        /**
         * @brief 点云滤波的结果
         */
        void filterResult(const Cloud::Ptr& cloud, float time);

    public slots:

        /**
         * @brief 在给定的 PointCloud 上组装本地 3D 网格，并对数据进行下采样 + 过滤
         * @param lx ly lz 设置体素网格叶大小
         * @param downsample 如果所有字段都需要下采样，则设置为 true，如果只是
         * XYZ，则设置为 false
         */
        void ApproximateVoxelGrid(float lx, float ly, float lz, bool downsample);

        /**
         * @brief 条件滤波器
         * @param val 设置过滤后的点是否应保留并设置为通过 setUserFilterValue
         * 给出的值（默认值：NaN）， 或者从 PointCloud
         * 中删除，从而可能破坏其组织结构。
         * @param val1 提供一个值，过滤点应设置为而不是删除它们。
         * @param con 设置过滤器将使用的条件
         */
        void ConditionalRemoval(Condition::Ptr con);

        /**
         * @brief 高斯滤波器
         * @param sigma 设置高斯的 sigma 参数
         * @param sigma_coefficient 设置相对于 sigma 因子的距离阈值
         * @param threshold 设置距离阈值如 pi，||pi - q|| > 不考虑阈值
         * @param radius 设置用于确定最近邻居的球体半径
         */
        void Convolution3D(float sigma, float sigma_coefficient, float threshold,
                           double radius);

        /**
         * @brief 允许用户过滤给定框内的所有数据的过滤器
         * @param min_pt 设置框的最小点
         * @param max_pt 设置框的最大点
         * @param translation 设置框的平移值
         * @param rotation 设置框的旋转值
         * @param transform 设置过滤前应应用于云的转换
         */
        void CropBox(const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt,
                     const Eigen::Affine3f& transform);

        /**
         * @brief 过滤位于 3D 闭合曲面或 2D 闭合多边形内部或外部的点
         * @param polygons 设置用于过滤点的船体顶点
         * @param dim 设置要使用的船体的维度
         * @param crop_outside 移除船体外部的点（默认）或船体内部的点
         */
        void CropHull(const std::vector<pcl::Vertices>& polygons, int dim,
                      bool crop_outside);

        /**
         * @brief 过滤由相机的姿势和视野给出的截锥体内的点
         * @param camera_pose 设置相机 w.r.t 原点的位姿
         * @param hfov 以度为单位设置相机的水平视野
         * @param vfov 以度为单位设置相机的垂直视野
         * @param np_dist 设置近平面距离
         * @param fp_dist 设置远平面距离
         */
        void FrustumCulling(const Eigen::Matrix4f& camera_pose, float hfov,
                            float vfov, float np_dist, float fp_dist);

        /**
         * @brief 在给定的 PointCloud 上组装一个本地 2D 网格，并对数据进行下采样
         * @param resolution 设置网格分辨率
         */
        void GridMinimum(const float resolution);

        /**
         * @brief 通过消除局部最大值来对云进行下采样
         * @param radius 设置用于确定点是否为局部最大值的半径
         */
        void LocalMaximum(float radius);

        /**
         * @brief 中值滤波器
         * @param window_size 设置过滤器的窗口大小
         * @param max_allowed_movement 设置一个dexel允许移动的最大值
         */
        void MedianFilter(int window_size, float max_allowed_movement);

        /**
         * @brief 移动最小二乘算法，用于数据平滑和改进的法线估计。
         * @param computer_normals 设置算法是否还应存储计算的法线
         * @param polynomial_order 设置要拟合的多项式的阶数
         * @param radius 设置用于确定用于拟合的 k 最近邻的球体半径。
         * @param sqr_gauss_param
         * 设置用于基于距离的邻居加权的参数（搜索半径的平方通常效果最好）
         * @param method1 设置要使用的上采样方法
         * @param method2 设置将点投影到 MLS 表面时使用的方法
         */
        void MovingLeastSquares(bool computer_normals, int polynomial_order,
                                float radius, double sqr_gauss_param,
                                int upsampling_method, double uradius,
                                double step_size, int dradius, float voxel_size,
                                int iterations, int projection_method);

        /**
         * @brief 在每个点计算的法线方向空间中对输入点云进行采样
         * @param sample 设置要采样的索引数
         * @param seed 设置随机函数的种子
         * @param binsx binsy binsz设置 x,y,z 方向的 bin 数量
         */
        void NormalSpaceSampling(int sample, int seed, int binsx, int binsy,
                                 int binsz);

        /**
         * @brief 直通滤波器
         * @param field_name 提供要用于过滤数据的字段的名称
         * @param limit_min 为过滤数据的字段设置数值限制
         * @param limit_max 为过滤数据的字段设置数值限制
         * @param negative  是否返回指定的限制间隔之外的数据
         */
        void PassThrough(const std::string& field_name, float limit_min,
                         float limit_max);

        /**
         * @brief 3D 平面剪裁器
         * @param plane_params 设置新的平面参数
         */
        void PlaneClipper3D(const Eigen::Vector4f& plane_params);

        /**
         * @brief 投影滤波器
         * @param type  要使用的模型类型（用户给定参数）
         * @param model 提供指向模型系数的指针
         * @param va 设置是返回所有数据，还是只返回投影的内点。
         */
        void ProjectInliers(int type, const pcl::ModelCoefficients::Ptr& model, bool va);

        /**
         * @brief 离群点滤波
         * @param radius 设置将确定哪些点是邻居的球体的半径
         * @param min_pts 设置为了被分类为内点而需要存在的邻居的数量
         */
        void RadiusOutlierRemoval(double radius, int min_pts);

        /**
         * @brief 具有均匀概率的随机抽样
         * @param sample 设置要采样的索引数
         * @param seed 设置随机函数的种子
         */
        void RandomSample(int sample, int seed);

        /**
         * @brief 将输入空间划分为网格，直到每个网格包含最多 N
         * 个点，并在每个网格内随机采样点
         * @param sample 设置每个网格中的最大样本数
         * @param seed 设置随机函数的种子
         * @param ratio 设置每个网格中要采样的点的比率
         */
        void SamplingSurfaceNormal(int sample, int seed, float ratio);

        /**
         * @brief 去除出现在边缘不连续处的鬼点
         * @param threshold 设置阴影点拒绝的阈值
         */
        void ShadowPoints(float threshold);

        /**
         * @brief 统计滤波器
         * @param nr_k 设置用于平均距离估计的最近邻的数量
         * @param stddev_mult 设置距离阈值计算的标准偏差乘数
         */
        void StatisticalOutlierRemoval(int nr_k, double stddev_mult);

        /**
         * @brief 在给定的 PointCloud 上组装一个本地 3D 网格，并对数据进行下采样 +
         * 过滤
         * @param radius 设置 3D 网格叶子大小
         */
        void UniformSampling(double radius);

        /**
         * @brief 在给定的 PointCloud 上组装一个本地 3D 网格，并对数据进行下采样 +
         * 过滤
         * @param lx ly lz 设置体素网格叶大小
         * @param downsample 如果所有字段都需要下采样，则设置为 true，如果只是
         * XYZ，则设置为 false
         * @param min_points_per_voxel 设置要使用的体素所需的最小点数
         */
        void VoxelGrid(float lx, float ly, float lz, bool downsample,
                       int min_points_per_voxel);
    };
} // namespace ct

#endif // CT_MODULES_FILTERS_H
