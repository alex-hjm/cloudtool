#ifndef SETPATH_H
#define SETPATH_H

#include <QDockWidget>
#include "common/cloudtree.h"
#include "common/tool.h"
namespace Ui {
class SetPath;
}

class SetPath : public QDockWidget
{
    Q_OBJECT

public:
    explicit SetPath(QWidget *parent = nullptr);
    ~SetPath();
    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void useCenter();
    void pickPoint();
    void preview();
    void add();
    void clear();
    void reset();

signals:
    void affine3f(const Eigen::Affine3f&);

public slots:
    void mouseLeftPressed(const Point2D&);
    void adjustEnable(bool state);
    void adjustPickPos(const Eigen::Affine3f&);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::SetPath *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    Eigen::Affine3f affine;
    bool is_picking;
};

#endif // SETPATH_H
