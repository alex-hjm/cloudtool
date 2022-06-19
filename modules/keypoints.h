/**
 * @file keypoints.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#ifndef CT_MODULES_KEYPOINTS_H
#define CT_MODULES_KEYPOINTS_H

#include <QObject>

#include "base/cloud.h"

#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>

namespace ct
{
    typedef pcl::PointXYZI      PointXYZI;
    typedef pcl::PointXYZRGB    PointXYZRGB;
    typedef pcl::PointWithScale PointWithScale;
    typedef pcl::RangeImage     RangeImage;

    class CT_EXPORT Keypoints : public QObject
    {
        Q_OBJECT
    public:
        explicit Keypoints(QObject* parent = nullptr)
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
         * @brief 关键点计算结果
         */
        void keypointsResult(const Cloud::Ptr& cloud, float time);

    public slots:

        /**
         * @brief 通过一幅距离图像输出关键点
         * @param range_image 深度图
         * @param support_size 支持的大小
         */
        void NarfKeypoint(const RangeImage::Ptr range_image, float support_size);

        /**
         * @brief 使用 2D Harris 关键点的思想，但不是使用图像梯度，而是使用表面法线
         * @param response_method 设置要计算的响应的方法:
         *                        1-HARRIS 2-NOBLE 3-LOWE 4-TOMASI 5-CURVATURE
         * @param radius 设置正常估计和非最大值抑制的半径
         * @param threshold 设置检测角的阈值
         * @param non_maxima 是否应应用非最大值抑制或应返回每个点的响应
         * @param do_refine 检测到的关键点是否需要细化
         */
        void HarrisKeypoint3D(int response_method, float threshold, bool non_maxima, bool do_refine);

        /**
         * @brief 检测给定点云的内在形状特征关键点
         * @param resolution 点云分辨率
         * @param gamma_21 设置第二个和第一个特征值之间比率的上限
         * @param gamma_32 设置第三个和第二个特征值之间比率的上限
         * @param min_neighbors 设置应用非最大值抑制算法时必须找到的最小邻居数
         * @param angle 设置将点标记为边界或规则的决策边界（角度阈值）
         */
        void ISSKeypoint3D(double resolution, double gamma_21, double gamma_32,
                           int min_neighbors, float angle);

        /**
         * @brief 检测包含点和强度的给定点云数据集的尺度不变特征变换关键点
         * @param min_scale 尺度空间中最小尺度的标准差
         * @param nr_octaves 要计算的八度音阶数（即倍频）
         * @param nr_scales_per_octave 在每个八度音程内计算的音阶数
         * @param min_contrast 检测所需的最小对比度
         */
        void SIFTKeypoint(float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast);

        /**
         * @brief 使用几何信息在点云上实现 Trajkovic 和 Hedley 角点检测器
         * @param compute_method 设置要计算的响应的方法
         * @param window_size 设置窗口大小
         * @param frist_threshold 设置第一个阈值以拒绝简单角计算阶段的角
         * @param second_threshold 设置第二个阈值以在最终角点计算阶段拒绝角点。
         */
        void TrajkovicKeypoint3D(int compute_method, int window_size, float frist_threshold, float second_threshold);
    };

}  // namespace pca

#endif  // PCA_KEYPOINTS_H
