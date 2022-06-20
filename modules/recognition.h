/**
 * @file recognition.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#ifndef CT_MODULES_RECOGNITION_H
#define CT_MODULES_RECOGNITION_H

#include <QObject>

#include "base/cloud.h"
#include <pcl/correspondence.h>

namespace ct
{
    struct Clusters
    {
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>translations;
        std::vector<pcl::Correspondences> clustered_corrs;
    };

    class CT_EXPORT Recognition : public QObject
    {
        Q_OBJECT
    public:
        explicit Recognition(QObject* parent = nullptr)
            : QObject(parent), scene_(nullptr), model_(nullptr), corrs_(nullptr)
        {}

        /**
         * @brief 设置场景点云
         */
        void setSceneCloud(const Cloud::Ptr& scene) { scene_ = scene; }

        /**
         * @brief 设置模型点云
         */
        void setModelCloud(const Cloud::Ptr& model) { model_ = model; }

        /**
         * @brief 提供指向输入数据集中的点与场景数据集中的点之间预先计算的对应关系的指针
         */
        void setModelSceneCorrespondences(const pcl::CorrespondencesConstPtr& corrs)
        {
            corrs_ = corrs;
        }

        /**
         * @brief 提供指向场景数据集参考帧的指针
         */
        void setSceneRf(const pcl::PointCloud<pcl::ReferenceFrame>::Ptr& scene_rf)
        {
            scene_rf_ = scene_rf;
        }

        /**
         * @brief 提供指向模型数据集参考帧的指针
         */
        void setModelRf(const pcl::PointCloud<pcl::ReferenceFrame>::Ptr& model_rf)
        {
            model_rf_ = model_rf;
        }

    private:
        Cloud::Ptr scene_;
        Cloud::Ptr model_;
        pcl::CorrespondencesConstPtr corrs_;
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf_;
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf_;

    signals:

        void recognitionResult(const Clusters& surface, float time);

    public slots:

        /**
         * @brief 实现 3D 对应分组的类，在特征对应之间强制几何一致性
         * @param threshold 设置最小集群大小
         * @param gc_size 设置共识集分辨率
         */
        void GeometricConsistencyGrouping(int threshold, double gc_size);

        /**
         * @brief 实现 3D
         * 对应分组算法的类，该算法可以处理在给定场景中找到的模型模板的多个实例
         * @param threshold
         * 设置将模型实例的存在推断到场景云中所需的霍夫空间中的最小投票数
         * @param bin_size 将每个 bin 的大小设置为 Hough 空间
         * @param use_interpolation 设置投票程序是否在霍夫空间的相邻箱之间插入分数
         * @param use_distance_weight 设置投票程序是否使用对应的距离作为分数
         * @param normals_radius
         * 如果尚未为模型云或场景云设置本地参考框架，则此算法会自行进行计算，
         *                      但需要合适的搜索半径来计算法线，以便随后计算
         * RF（如果未设置默认的 15 个最近邻居搜索）。
         * @param rf_adius 如果尚未为模型云或场景云设置本地参考框架，
         *                  则此算法会自行进行计算，但需要合适的搜索半径才能执行此操作
         */
        void Hough3DGrouping(double threshold, double bin_size,
                             bool use_interpolation, bool use_distance_weight,
                             float normals_radius, float rf_adius);
    };

}  // namespace pca

#endif  // PCA_RECOGNITION_H
