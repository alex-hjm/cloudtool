#ifndef MEASURE_H
#define MEASURE_H

#include <QDialog>
#include "src/common/cloudtree.h"

namespace Ui {
class Measure;
}

class Measure : public QDialog
{
    Q_OBJECT

public:
    explicit Measure(QWidget *parent = nullptr);
    ~Measure();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void start();
    void reset();

public slots:
    void leftPressPoint(Point2D);
    void leftReleasePoint(Point2D);
    void rightReleasePoint(Point2D);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Measure *ui;
    bool isMeasuring;
    bool pickStart;
    //Cloud selectedCloud;
    //CloudXYZRGBN::Ptr pick_point;
    //std::vector<Cloud> allClouds;
    Point2D currentpt;
    PointXYZRGBN start_point;
    PointXYZRGBN end_point;
    //std::vector<Point2D> points;
    //std::vector<CloudXYZRGBN::Ptr> measurePoints;
};

#endif // MEASURE_H
