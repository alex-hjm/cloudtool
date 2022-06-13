#ifndef SEGMENT_H
#define SEGMENT_H

#include <QDialog>
#include "src/common/cloudtree.h"

namespace Ui {
class Segment;
}

class Segment : public QDialog
{
    Q_OBJECT

public:
    explicit Segment(QWidget *parent = nullptr);
    ~Segment();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void selectIn();
    void selectOut();
    void add();
    void apply();
    void start();
    void reset();

protected:
    void closeEvent(QCloseEvent *event);

public slots:
    void leftPressPoint(Point2D);
    void leftReleasePoint(Point2D);
    void rightReleasePoint(Point2D);
    void movePoint(Point2D);

private:
    Ui::Segment *ui;

    int pickType;
    bool isPicking;
    bool pickStart;

    CloudXYZRGBN::Ptr segCloud;
    std::vector<int> indices;
    std::vector<Point2D> points;
};

#endif // SEGMENT_H
