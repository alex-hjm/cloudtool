#ifndef PCA_SURFACE_H
#define PCA_SURFACE_H

#include <QObject>

#include "pca/cloudview.h"
#include "pca/exports.h"

namespace pca {
class PCA_EXPORT Surface : public QObject {
  Q_OBJECT
 public:
  explicit Surface(QObject *parent = nullptr) : QObject(parent), cloud_() {}

  /**
   * @brief 设置输入点云
   */
  void setInputCloud(const Cloud::Ptr &cloud) { cloud_ = cloud; }

 private:
  Cloud::Ptr cloud_;

 signals:

  void surfaceResult(const pcl::PolygonMesh::Ptr &surface, float time);

 public slots:

  /**
   * @brief 使用 libqhull 库的 ConcaveHull（alpha 形状）
   * @param alpha 设置 alpha 值，该值限制生成的船体段的大小（船体越小越详细）
   * @param value 如果 keep_information_is 设置为
   * true，则凸包点会保留其他信息，例如 rgb、法线
   * @param dimensio 设置输入数据的维度，2D 或 3D。
   */
  void ConcaveHull(double alpha, bool value, int dimensio);

  /**
   * @brief ConvexHull 使用 libqhull 库
   * @param value 如果设置为 true，则调用 qhull 库来计算凸包的总面积和体积
   * @param dimensio 设置输入数据的维度，2D 或 3D
   */
  void ConvexHull(bool value, int dimensio);

  /**
   * @brief 剪耳三角测量算法
   */
  void EarClipping(pcl::PolygonMesh::Ptr &surface);

  /**
   * @brief 基于局部 2D 投影的 3D 点的贪心三角剖分算法的实现
   * @param mu
   * 设置最近邻距离的乘数，得到每个点的最终搜索半径（这将使算法适应云中不同的点密度）
   * @param nnn 设置要搜索的最近邻居的最大数量
   * @param radius 设置用于确定用于三角剖分的 k 最近邻的球体半径
   * @param min 设置每个三角形应具有的最小角度
   * @param max 设置每个三角形可以有的最大角度
   * @param ep 如果点的法线与查询点法线的偏差大于此值，则不要考虑进行三角测量
   * @param consistent 如果输入法线方向一致，则设置该标志
   * @param consistent_ordering
   * 设置标志以一致地对生成的三角形顶点进行排序（法线周围的正方向）
   */
  void GreedyProjectionTriangulation(double mu, int nnn, double radius, int min,
                                     int max, double ep, bool consistent,
                                     bool consistent_ordering);

  /**
   * @brief 网格投影面重建方法
   * @param resolution 设置网格单元格的大小
   * @param padding_size
   * 在对向量进行平均时，我们找到填充区域内所有输入数据点的并集，并进行加权平均。
   * @param k 仅在使用 k 最近邻搜索而不是查找点联合时才设置此项
   * @param max_binary_search_level 二分搜索用于投影
   */
  void GridProjection(double resolution, int padding_size, int k,
                      int max_binary_search_level);

  /**
   * @brief 行进立方体表面重建算法，使用基于切平面距离的有符号距离函数，由 Hoppe
   * 等提出
   * @param iso_level 设置要提取的表面的 iso 级别的方法
   * @param res_x res_y res_z 设置行进立方体网格分辨率的方法
   * @param percentage
   * 设置参数的方法，该参数定义在点云的边界框和网格限制之间的网格内应保留多少可用空间
   * @param dist_ignore 设置忽略远离点云的体素的距离的方法
   */
  void MarchingCubesHoppe(float iso_level, int res_x, int res_y, int res_z,
                          float percentage, float dist_ignore);

  /**
   * @brief 行进立方体表面重建算法，使用基于径向基函数的有符号距离函数
   * @param iso_level 设置要提取的表面的 iso 级别的方法
   * @param res_x res_y res_z 设置行进立方体网格分辨率的方法
   * @param percentage
   * 设置参数的方法，该参数定义在点云的边界框和网格限制之间的网格内应保留多少可用空间
   * @param epsilon 设置离面点位移值
   */
  void MarchingCubesRBF(float iso_level, int res_x, int res_y, int res_z,
                        float percentage, float epsilon);

  /**
   * @brief 泊松曲面重建算法
   * @param depth 设置将用于表面重建的树的最大深度
   * @param min_depth 设置将用于表面重建的树的最小深度
   * @param point_weight 设置点的权重
   * @param scale 设置用于重建的立方体直径与样本边界立方体直径之间的比率
   * @param solver_divide 设置块 Gauss-Seidel 求解器用于求解拉普拉斯方程的深度
   * @param iso_divide 设置应使用块等值面提取器提取等值面的深度
   * @param samples_per_node
   * 当八叉树结构适应采样密度时，设置应落在八叉树节点内的最小采样点数
   * @param confidence 设置置信度标志
   * @param output_polygons 启用此标志会告诉重建器输出多边形网格（而不是对
   * Marching Cubes 的结果进行三角测量）
   * @param manifold 设置歧管标志
   */
  void Poisson(int depth, int min_depth, float point_weight, float scale,
               int solver_divide, int iso_divide, float samples_per_node,
               bool confidence, bool output_polygons, bool manifold);
};
}  // namespace pca

#endif  // PCA_SURFACE_H
