/**
 * @file pickpoints.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_TOOL_PICKPOINTS_H
#define CT_TOOL_PICKPOINTS_H

#include "base/customdialog.h"
#include "base/common.h"
namespace Ui
{
    class PickPoints;
}

class PickPoints : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit PickPoints(QWidget* parent = nullptr);
    ~PickPoints();

    virtual void init();
    void start();
    void add();
    virtual void reset();
    virtual void deinit() { m_cloudview->clearInfo(); }

private:
    void updateInfo(int index);

public slots:
    void mouseLeftPressed(const ct::PointXY& pt);
    void mouseLeftReleased(const ct::PointXY& pt);
    void mouseRightReleased(const ct::PointXY& pt);
    void mouseMoved(const ct::PointXY& pt);

private:
    Ui::PickPoints* ui;
    bool is_picking;
    bool pick_start;
    ct::Cloud::Ptr m_selected_cloud;
    ct::Cloud::Ptr m_pick_cloud;
    ct::PointXY m_pick_point;
};

#endif // CT_TOOL_PICKPOINTS_H
