#ifndef MEASURE_H
#define MEASURE_H

#include <QDialog>
#include "common/cloudtree.h"

namespace Ui {
class Measure;
}

class Measure : public QDialog
{
    Q_OBJECT

public:
    explicit Measure(QWidget *parent = nullptr);
    ~Measure();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void start();
    void reset();

public slots:
    void leftPressPoint(const Point2D&);
    void leftReleasePoint(const Point2D&);
    void rightReleasePoint(const Point2D&);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Measure *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    bool is_measuring;
    bool pick_start;
    Point2D current_pt;
    Cloud::Ptr start_pt;
    Cloud::Ptr end_pt;
};

#endif // MEASURE_H
