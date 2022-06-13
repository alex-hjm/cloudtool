#ifndef CUTTING_H
#define CUTTING_H

#include <QDialog>
#include "common/cloudtree.h"

namespace Ui {
class Cutting;
}

class Cutting : public QDialog
{
    Q_OBJECT

public:
    explicit Cutting(QWidget *parent = nullptr);
    ~Cutting();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void selectIn();
    void selectOut();
    void add();
    void apply();
    void start();
    void reset();

public slots:
    void leftPressPoint(const Point2D &pt);
    void leftReleasePoint(const Point2D &pt);
    void rightReleasePoint(const Point2D &pt);
    void movePoint(const Point2D &pt);

private :
    void cuttingCloud(bool selectIn);
protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Cutting *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;

    int pick_type;
    bool is_picking;
    bool pick_start;

    Cloud::Ptr cutted_cloud;
    std::vector<int> indices;
    std::vector<Point2D> points;
};

#endif // CUTTING_H
