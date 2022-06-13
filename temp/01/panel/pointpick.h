#ifndef POINTPICK_H
#define POINTPICK_H

#include <QDialog>
#include "common/cloudtree.h"

namespace Ui {
class PointPick;
}

class PointPick : public QDialog
{
    Q_OBJECT

public:
    explicit PointPick(QWidget *parent = nullptr);
    ~PointPick();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void add();
    void start();
    void reset();

public slots:
    void leftReleasePoint(const Point2D&);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::PointPick *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    bool is_picking;
    Cloud::Ptr pick_cloud;
};

#endif // POINTPICK_H
