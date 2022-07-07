/**
 * @file cutting.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_TOOL_CUTTING_H
#define CT_TOOL_CUTTING_H

#include "base/customdialog.h"

namespace Ui
{
    class Cutting;
}

class Cutting : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit Cutting(QWidget* parent = nullptr);
    ~Cutting();

    virtual void init();
    void selectIn();
    void selectOut();
    void add();
    void apply();
    void start();
    virtual void reset();
    virtual void deinit() { m_cloudview->clearInfo(); }

private:
    void updateInfo(int index);
    void cuttingCloud(bool select_in);

public slots:
    void mouseLeftPressed(const ct::PointXY& pt);
    void mouseLeftReleased(const ct::PointXY& pt);
    void mouseRightReleased(const ct::PointXY& pt);
    void mouseMoved(const ct::PointXY& pt);

private:
    Ui::Cutting* ui;
    bool is_picking;
    bool pick_start;
    std::vector<ct::PointXY> m_pick_points;
    std::map<QString, ct::Cloud::Ptr> m_cutting_map;
};

#endif // CUTTING_H
