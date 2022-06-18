/**
 * @file filters.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_TOOL_FILTERS_H
#define CT_TOOL_FILTERS_H

#include <QThread>

#include "base/customdock.h"

#include "modules/filters.h"

namespace Ui
{
    class Filters;
}

class Filters : public ct::CustomDock
{
    Q_OBJECT

public:
    explicit Filters(QWidget* parent = nullptr);
    ~Filters();

    void preview();
    void add();
    void apply();
    virtual void reset();

signals:
    void passThrough(const std::string& field_name, float limit_min, float limit_max);
    void voxelGrid(float lx, float ly, float lz);
    void approximateVoxelGrid(float lx, float ly, float lz);
    void statisticalOutlierRemoval(int nr_k, double stddev_mult);
    void radiusOutlierRemoval(double radius, int min_pts);
    void conditionalRemoval(ct::ConditionBase::Ptr con);
    void movingLeastSquares(bool computer_normals, int polynomial_order, float radius);
    void gridMinimum(const float resolution);
    void localMaximum(float radius);
    void shadowPoints(float threshold);

public slots:
    void filterResult(const ct::Cloud::Ptr& cloud, float time);

private:
    ct::ConditionBase::Ptr getCondition();
    void getRange(int index);

private:
    Ui::Filters* ui;
    QThread m_thread;
    ct::Filters* m_filters;
    std::unordered_map<QString, ct::Cloud::Ptr> m_filter_map;
};

#endif  // CT_TOOL_FILTERS_H
