/**
 * @file registration.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#ifndef CT_MODULES_REGISTRATION_H
#define CT_MODULES_REGISTRATION_H

#include "base/cloud.h"
#include "base/exports.h"
//#include "modules/features.h"

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transformation_estimation.h>

#include <QObject>

namespace ct
{
    typedef pcl::registration::TransformationEstimation<PointXYZRGBN, PointXYZRGBN, float>      TransEst; /*表示转换估计方法的类*/
    typedef pcl::registration::CorrespondenceEstimationBase<PointXYZRGBN, PointXYZRGBN, float>  CorreEst; /*表示对应估计方法的类*/
    typedef pcl::registration::CorrespondenceRejector                                           CorreRej; /*表示对应拒绝方法的类*/

    class CT_EXPORT Registration : public QObject
    {
        Q_OBJECT
    public:
        explicit Registration(QObject* parent = nullptr)
            : QObject(parent),
            target_cloud_(new Cloud),
            source_cloud_(new Cloud),
            corr(new pcl::Correspondences),
            te_(nullptr),
            ce_(nullptr),
            nr_iterations_(10),
            ransac_iterations_(0),
            inlier_threshold_(0.05),
            distance_threshold_(std::sqrt(std::numeric_limits<double>::max())),
            transformation_epsilon_(0.0),
            transformation_rotation_epsilon_(0.0),
            euclidean_fitness_epsilon_(-std::numeric_limits<double>::max())
        {}

        /**
         * @brief 设置输入源点云
         */
        void setInputSource(const Cloud::Ptr& cloud) { source_cloud_ = cloud; }

        /**
         * @brief 设置输入目标点云
         */
        void setInputTarget(const Cloud::Ptr& cloud) { target_cloud_ = cloud; }

        /**
         * @brief 提供指向变换估计对象的指针
         */
        void setTransformationEstimation(const TransEst::Ptr& te) { te_ = te; }

        /**
         * @brief 提供指向对应估计对象的指针
         */
        void setCorrespondenceEstimation(const CorreEst::Ptr& ce) { ce_ = ce; }

        /**
         * @brief 将新的对应拒绝器添加到列表中
         */
        void addCorrespondenceRejector(const QString& name, CorreRej::Ptr& cr)
        {
            cr_map[name] = cr;
        }

        /**
         * @brief 删除列表中的对应拒绝器
         */
        void removeCorrespondenceRejector(const QString& name)
        {
            if (cr_map.find(name) != cr_map.end()) cr_map.erase(name);
        }

        /**
         * @brief 设置内部优化应该运行的最大迭代次数
         */
        void setMaximumIterations(int nr_iterations)
        {
            nr_iterations_ = nr_iterations;
        }

        /**
         * @brief 设置 RANSAC 应该运行的迭代次数
         */
        void setRANSACIterations(int ransac_iterations)
        {
            ransac_iterations_ = ransac_iterations;
        }

        /**
         * @brief 设置内部 RANSAC 离群值拒绝循环的离群距离阈值
         */
        void setRANSACOutlierRejectionThreshold(double inlier_threshold)
        {
            inlier_threshold_ = inlier_threshold;
        }

        /**
         * @brief 设置源 <-> 目标中两个对应点之间的最大距离阈值
         */
        void setMaxCorrespondenceDistance(double distance_threshold)
        {
            distance_threshold_ = distance_threshold;
        }

        /**
         * @brief 设置转换精度（两个连续转换之间的最大允许平移平方差）
         */
        void setTransformationEpsilon(double epsilon)
        {
            transformation_epsilon_ = epsilon;
        }

        /**
         * @brief 设置变换旋转精度（两个连续变换之间的最大允许旋转差）
         */
        void setTransformationRotationEpsilon(double epsilon)
        {
            transformation_rotation_epsilon_ = epsilon;
        }

        /**
         * @brief 在认为算法已经收敛之前，设置 ICP
         * 循环中两个连续步骤之间允许的最大欧几里得误差
         */
        void setEuclideanFitnessEpsilon(double epsilon)
        {
            euclidean_fitness_epsilon_ = epsilon;
        }

    private:
        Cloud::Ptr target_cloud_;
        Cloud::Ptr source_cloud_;
        pcl::CorrespondencesPtr corr;
        TransEst::Ptr te_;
        CorreEst::Ptr ce_;
        std::unordered_map<QString, CorreRej::Ptr> cr_map;
        int nr_iterations_;
        int ransac_iterations_;
        double inlier_threshold_;
        double distance_threshold_;
        double transformation_epsilon_;
        double transformation_rotation_epsilon_;
        double euclidean_fitness_epsilon_;

    signals:

        void registrationResult(bool success, const Cloud::Ptr& ail_cloud, double score, const Eigen::Matrix4f& matrix, float time);

        void correspondenceEstimationResult(const pcl::CorrespondencesPtr& corr, float time);

        void correspondenceRejectorResult(const pcl::CorrespondencesPtr& corr, float time);

        void transformationEstimationResult(const Eigen::Matrix4f& matrix, float time);

    public:
        /**
         * @brief 表示用于确定目标和查询点集/特征之间对应关系的基类
         */
        template <typename Type> // point or feature
        void CorrespondenceEstimation(const typename pcl::PointCloud<Type>::Ptr& source,
                                      const typename pcl::PointCloud<Type>::Ptr& target)
        {
            TicToc time;
            time.tic();
            pcl::registration::CorrespondenceEstimation<Type, Type, float> ce;
            ce.setInputSource(source);
            ce.setInputTarget(target);
            KdTree<Type>::Ptr target_tree(new KdTree<Type>);
            KdTree<Type>::Ptr source_tree(new KdTree<Type>);
            ce.setSearchMethodSource(source_tree);
            ce.setSearchMethodTarget(target_tree);
            ce.determineCorrespondences(*corr);
            emit correspondenceEstimationResult(corr, time.toc());
        }

        /**
         * @brief 基于一组特征描述符的对应拒绝方法。
         * @param thresh_src thresh_tar 任何高于此距离阈值的特征对应都将被视为不良并被过滤掉
         */
        template <typename Feature>
        void CorrespondenceRejectorFeatures(const typename pcl::PointCloud<Feature>::Ptr& source,
                                            const typename pcl::PointCloud<Feature>::Ptr& target,
                                            double thresh_src, double thresh_tar)
        {
            TicToc time;
            time.tic();
            KdTree<PointXYZRGBN>::Ptr target_tree(new KdTree<PointXYZRGBN>);

            pcl::registration::CorrespondenceRejectorFeatures cj;
            cj.setSourceFeature<Feature>(source, source_cloud_->id().toStdString());
            cj.setTargetFeature<Feature>(target, target_cloud_->id().toStdString());
            cj.setSourceNormals(source_cloud_);
            cj.setTargetNormals(target_cloud_);
            cj.setDistanceThreshold(thresh_src, source_cloud_->id().toStdString());
            cj.setDistanceThreshold(thresh_tar, target_cloud_->id().toStdString());
            cj.setInputCorrespondences(corr);
            cj.getCorrespondences(corr);
            emit correspondenceRejectorResult(corr, time.toc());
        }

        /**
         * @brief 用于 3D 配准的快速点特征直方图 (FPFH) 的初始对齐算法
         * @param fearure_type 特征描述符类型 0-PFH 1-FPFH 2-VFH 3-SHOT
         * @param min_sample_distance 设置样本之间的最小距离。
         * @param nr_samples 设置每次迭代期间要使用的样本数
         * @param k 设置选择随机特征对应时要使用的邻居数
         */
        template <typename Feature>
        void SampleConsensusInitialAlignment(const typename pcl::PointCloud<Feature>::Ptr& source,
                                             const typename pcl::PointCloud<Feature>::Ptr& target,
                                             float min_sample_distance, int nr_samples, int k)
        {
            TicToc time;
            time.tic();
            Cloud::Ptr ail_cloud(new Cloud);
            pcl::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::KdTree<PointXYZRGBN>);
            pcl::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::KdTree<PointXYZRGBN>);
            pcl::SampleConsensusInitialAlignment<PointXYZRGBN, PointXYZRGBN, Feature> reg;

            reg.setInputTarget(target_cloud_);
            reg.setInputSource(source_cloud_);
            reg.setSourceFeatures(source);
            reg.setTargetFeatures(target);
            reg.setSearchMethodTarget(target_tree);
            reg.setSearchMethodSource(source_tree);
            reg.setTransformationEstimation(te_);
            reg.setCorrespondenceEstimation(ce_);
            for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj->second);
            reg.setMaximumIterations(nr_iterations_);
            reg.setRANSACIterations(ransac_iterations_);
            reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
            reg.setMaxCorrespondenceDistance(distance_threshold_);
            reg.setTransformationEpsilon(transformation_epsilon_);
            reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
            reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

            reg.setCorrespondenceRandomness(k);
            reg.setMinSampleDistance(min_sample_distance);
            reg.setNumberOfSamples(nr_samples);

            reg.align(*ail_cloud);
            emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                    reg.getFinalTransformation().cast<float>(), time.toc());
        }

        /**
         * @brief 使用前排斥 RANSAC 例程的姿势估计和对齐类
         * @param fearure_type  特征描述符类型 0-PFH 1-FPFH 2-VFH 3-SHOT
         * @param nr_samples 设置每次迭代期间要使用的样本数
         * @param k 设置选择随机特征对应时要使用的邻居数
         * @param similarity_threshold 在 [0,1] 中设置底层多边形对应拒绝器对象的边缘长度之间的相似度阈值，其中 1 是完美匹配。
         * @param inlier_fraction 设置所需的内部分数（输入的）
         */
        template <typename Feature>
        void SampleConsensusPrerejective(const typename pcl::PointCloud<Feature>::Ptr& source,
                                         const typename pcl::PointCloud<Feature>::Ptr& target,
                                         int nr_samples, int k, float similarity_threshold, float inlier_fraction)
        {
            TicToc time;
            time.tic();
            Cloud::Ptr ail_cloud(new Cloud);
            pcl::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::KdTree<PointXYZRGBN>);
            pcl::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::KdTree<PointXYZRGBN>);
            pcl::SampleConsensusPrerejective<PointXYZRGBN, PointXYZRGBN, Feature> reg;

            reg.setInputTarget(target_cloud_);
            reg.setInputSource(source_cloud_);
            reg.setSourceFeatures(source);
            reg.setTargetFeatures(target);
            reg.setSearchMethodTarget(target_tree);
            reg.setSearchMethodSource(source_tree);
            reg.setTransformationEstimation(te_);
            reg.setCorrespondenceEstimation(ce_);
            for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj->second);
            reg.setMaximumIterations(nr_iterations_);
            reg.setRANSACIterations(ransac_iterations_);
            reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
            reg.setMaxCorrespondenceDistance(distance_threshold_);
            reg.setTransformationEpsilon(transformation_epsilon_);
            reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
            reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

            reg.setCorrespondenceRandomness(k);
            reg.setSimilarityThreshold(similarity_threshold);
            reg.setNumberOfSamples(nr_samples);
            reg.setInlierFraction(inlier_fraction);

            reg.align(*ail_cloud);
            emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                    reg.getFinalTransformation().cast<float>(), time.toc());
        }

    public slots:

        /**
         * @brief 将对应关系计算为目标云中具有最小值的点
         * @param k 设置目标点云中要考虑的最近邻的数量
         */
        void CorrespondenceEstimationBackProjection(int k);

        /**
         * @brief 将对应关系计算为目标云中与在输入云上计算的法线具有最小距离的点
         * @param k 设置目标点云中要考虑的最近邻的数量
         */
        void CorrespondenceEstimationNormalShooting(int k);

        /**
         * @brief 通过使用相机内部和外部参数将源点云投影到目标点云上来计算对应关系
         * @param fx fy 设置目标相机的焦距参数
         * @param cx cy 设置目标相机的相机中心参数
         * @param src_to_tgt_trans 设置从源点云到目标点云的转换
         * @param depth_threshold 设置深度阈值；在将源点投影到目标相机的图像空间后，
         *                        将该阈值应用于相应的dexels的深度，以拒绝彼此相距太远的dexels。
         */
        void CorrespondenceEstimationOrganizedProjection(float fx, float fy, float cx, float cy,
                                                         const Eigen::Matrix4f& src_to_tgt_trans, float depth_threshold);

        /**
         * @brief 计算输入和目标云中对应点之间对应分数的接口
         */
        double DataContainer(const pcl::Correspondence& corr, bool from_normals);

        /**
         * @brief 基于对对应关系之间的距离进行阈值处理的简单对应拒绝方法。
         * @param distance 设置用于在对应拒绝中进行阈值处理的最大距离。
         */
        void CorrespondenceRejectorDistance(float distance);

        /**
         * @brief 基于对应关系中值距离的阈值处理的简单对应拒绝方法
         * @param factor 设置拒绝对应的因素
         */
        void CorrespondenceRejectorMedianDistance(double factor);

        /**
         * @brief 基于拒绝对应关系中重复匹配索引的对应拒绝方法
         */
        void CorrespondenceRejectorOneToOne();

        /**
         * @brief 简单的对应拒绝方法
         */
        void CorrespondenceRejectionOrganizedBoundary(int val);

        /**
         * @brief 实现了一种对应拒绝方法，该方法利用输入对应关系在每个模型上形成用户可指定
         *        基数的虚拟多边形，从而利用两个点集之间的低级和姿势不变几何约束。
         * @param cardinality 设置多边形基数
         * @param similarity_threshold 在 [0,1[ 中设置边缘长度之间的相似度阈值，其中 1 是完美匹配
         * @param iterations 设置迭代次数
         */
        void CorrespondenceRejectorPoly(int cardinality, float similarity_threshold, int iterations);

        /**
         * @brief 使用随机样本共识实现对应拒绝以识别异常值（并拒绝异常值）
         * @param threshold 设置对应点之间的最大距离
         * @param max_iterations 设置最大迭代次数
         * @param refine 指定是否应使用内点的方差在内部细化模型
         */
        void CorrespondenceRejectorSampleConsensus(double threshold, int max_iterations, bool refine);

        /**
         * @brief 基于对应点法线之间夹角的简单对应拒绝方法
         * @param threshold 设置法线之间的阈值角度以进行对应拒绝
         */
        void CorrespondenceRejectorSurfaceNormal(double threshold);

        /**
         * @brief 实现了类 ICP 配准算法的对应拒绝，该算法仅使用最佳的“k”对应关系，
         *        其中“k”是对两个被注册的点云之间重叠的估计。
         * @param ratio 设置点云之间的预期重叠比率（根据对应关系）
         * @param min_corre 设置最小对应数量
         */
        void CorrespondenceRejectorTrimmed(float ratio, int min_corre);

        /**
         * @brief 通过将一定百分比的距离最短的对应视为内点来实现简单的对应拒绝方法
         * @param min_ratio 简要设置最小重叠率
         * @param min_corre 简要设置最大重叠率
         */
        void CorrespondenceRejectorVarTrimmed(double min_ratio, double max_ratio);

        /**
         * @brief 广义迭代最近点算法
         * @param k 设置选择点邻域以计算协方差时使用的邻域数
         * @param max 在优化步骤设置最大迭代次数
         * @param tra_tolerance 设置早期优化停止的最小平移梯度阈值
         * @param rol_tolerance 设置早期优化停止的最小旋转梯度阈值
         * @param use_recip_corre 设置是否使用互惠对应
         */
        void GeneralizedIterativeClosestPoint(int k, int max, double tra_tolerance, double rol_tolerance,
                                              bool use_recip_corre);

        /**
         * @brief 用于鲁棒成对表面配准的四点全等集
         * @param delta 设置对内部计算的参数进行加权的常数因子
         * @param normalize 是否应根据点云密度对 delta 进行归一化
         * @param approx_overlap 设置源和目标之间的近似重叠
         * @param score_threshold 设置用于早期完成方法的评分阈值。
         * @param nr_samples 设置对齐期间要使用的源样本数
         * @param max_norm_diff 以度为单位设置有效点对应之间的最大法线差。
         * @param max_runtime 以秒为单位设置最大计算时间
         */
        void FPCSInitialAlignment(float delta, bool normalize, float approx_overlap,
                                  float score_threshold, int nr_samples,
                                  float max_norm_diff, int max_runtime);

        /**
         * @brief 基于关键点的四点全等集的无标记点云配准
         * @param delta 设置对内部计算的参数进行加权的常数因子
         * @param normalize 是否应根据点云密度对 delta 进行归一化
         * @param approx_overlap 设置源和目标之间的近似重叠
         * @param score_threshold 设置用于早期完成方法的评分阈值。
         * @param nr_samples 设置对齐期间要使用的源样本数
         * @param max_norm_diff 以度为单位设置有效点对应之间的最大法线差。
         * @param max_runtime 以秒为单位设置最大计算时间
         * @param upper_trl_boundary 设置用于分数评估的较高平移阈值
         * @param lower_trl_boundary 设置用于分数评估的较低平移阈值
         * @param lambda 设置平移成本术语的权重系数。
         */
        void KFPCSInitialAlignment(float delta, bool normalize, float approx_overlap,
                                   float score_threshold, int nr_samples,
                                   float max_norm_diff, int max_runtime,
                                   float upper_trl_boundary, float lower_trl_boundary,
                                   float lambda);

        /**
         * @brief 迭代最近点算法
         * @param use_recip_corre 设置是否使用互惠对应
         */
        void IterativeClosestPoint(bool use_recip_corre);

        /**
         * @brief 使用基于点到平面距离估计的变换的迭代最近点算法
         * @param use_recip_corre 设置是否使用互惠对应
         * @param use_symmetric_objective 设置是否使用对称目标函数
         * @param enforce_same_direction_normals
         * 设置是否逐点否定源法线或目标法线，使它们指向同一方向
         */
        void IterativeClosestPointWithNormals(bool use_recip_corre, bool use_symmetric_objective,
                                              bool enforce_same_direction_normals);

        /**
         * @brief 使用 Levenberg-Marquardt 优化后端的 ICP 变体
         * @param use_recip_corre 设置是否使用互惠对应
         */
        void IterativeClosestPointNonLinear(bool use_recip_corre);

        /**
         * @brief 点云数据的 3D 正态分布变换配准实现
         * @param resolution 设置/更改体素网格分辨率
         * @param step_size 获取牛顿线搜索最大步长
         * @param outlier_ratio 设置/更改点云异常值比率
         */
        void NormalDistributionsTransform(float resolution, double step_size, double outlier_ratio);

        /**
         * @brief 为给定的数据集对实现简单的 2D 刚性变换估计 (x, y, theta)。
         */
        void TransformationEstimation2D();

        /**
         * @brief 表示用于转换估计的类，基于：
         *       - 3对的对应向量（平面情况）
         *       - 大小为 3 的两个点云（源和目标）
         *       - 具有一组 3 个索引的点云（源）和另一个点云（目标）
         *       - 具有两组大小为 3 的索引（源和目标）的两个点云
         */
        void TransformationEstimation3Point();

        /**
         * @brief 实现基于对偶四元数的变换估计，对齐给定的对应关系
         */
        void TransformationEstimationDualQuaternion();

        /**
         * @brief 实现了基于 Levenberg Marquardt 的对对齐给定对应关系的变换的估计
         */
        void TransformationEstimationLM();

        /**
         * @brief 使用 Levenberg Marquardt
         * 优化来找到最小化给定对应关系之间的点到平面距离的变换
         */
        void TransformationEstimationPointToPlane();

        /**
         * @brief 实现线性最小二乘 (LLS)
         * 近似，以最小化两个具有法线的对应点云之间的点到平面距离
         */
        void TransformationEstimationPointToPlaneLLS();

        /**
         * @brief 实现了一个线性最小二乘 (LLS)
         * 近似，用于最小化两个具有法线的对应点云之间的点到平面距离，
         *        并可以为对应关系分配权重
         */
        void TransformationEstimationPointToPlaneLLSWeighted();

        /**
         * @brief 使用 Levenberg Marquardt
         * 优化来找到最小化给定对应关系之间的点到平面距离的变换
         */
        void TransformationEstimationPointToPlaneWeighted();

        /**
         * @brief 实现对齐给定对应关系的基于 SVD 的变换估计
         */
        void TransformationEstimationSVD();

        /**
         * @brief 实现线性最小二乘 (LLS)
         * 近似，以最小化两个具有法线的对应点云之间的对称点到平面距离
         */
        void TransformationEstimationSymmetricPointToPlaneLLS(bool enforce_same_direction_normals);

        /**
         * @brief 计算源数据集和目标数据集之间的 L2SQR 范数
         */
        void TransformationValidationEuclidean();
    };
}  // namespace pca
#endif  // PCA_REGISTRATION_H
