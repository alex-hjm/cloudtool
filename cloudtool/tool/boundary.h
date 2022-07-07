/**
 * @file boundary.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_TOOL_BOUNDARY_H
#define CT_TOOL_BOUNDARY_H

#include "base/customdialog.h"

#include "modules/features.h"

#include <QThread>

namespace Ui
{
    class Boundary;
}

class Boundary : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit Boundary(QWidget* parent = nullptr);
    ~Boundary();

    void preview();
    void add();
    void apply();
    virtual void reset();

signals:
    void BoundaryEstimation(float angle);

public slots:
    void boundaryResult(const ct::Cloud::Ptr& cloud, float time);

private:
    Ui::Boundary* ui;
    QThread m_thread;
    ct::Features* m_feature;
    std::map<QString, ct::Cloud::Ptr> m_boundary_map;
};

#endif  // CT_TOOL_BOUNDARY_H
