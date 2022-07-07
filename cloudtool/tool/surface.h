/**
 * @file surface.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_TOOL_SURFACE_H
#define CT_TOOL_SURFACE_H

#include "base/customdock.h"

#include "modules/surface.h"

#include <QThread>

namespace Ui
{
    class Surface;
}

class Surface : public ct::CustomDock
{
    Q_OBJECT

public:
    explicit Surface(QWidget* parent = nullptr);
    ~Surface();

    void preview();
    void reset();

signals:
    void GreedyProjectionTriangulation(double mu, int nnn, double radius, int min, int max, double ep,
                                       bool consistent, bool consistent_ordering);
    void GridProjection(double resolution, int padding_size, int k, int max_binary_search_level);
    void Poisson(int depth, int min_depth, float point_weight, float scale,
                 int solver_divide, int iso_divide, float samples_per_node,
                 bool confidence, bool output_polygons, bool manifold);
    void MarchingCubesRBF(float iso_level, int res_x, int res_y, int res_z,
                          float percentage, float epsilon);
    void MarchingCubesHoppe(float iso_level, int res_x, int res_y, int res_z,
                            float percentage, float dist_ignore);
    void ConvexHull(bool value, int dimensio);
    void ConcaveHull(double alpha, bool value, int dimensio);

public slots:
    void surfaceResult(const QString& id, const ct::PolygonMesh::Ptr& surface, float time);

private:
    Ui::Surface* ui;
    QThread m_thread;
    ct::Surface* m_surface;
    std::map<QString, ct::PolygonMesh::Ptr> m_surface_map;
};

#endif // CT_TOOL_SURFACE_H
