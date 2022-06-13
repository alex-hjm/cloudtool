#ifndef KEYPOINT_H
#define KEYPOINT_H

#include <QWidget>

#include "src/common/processtree.h"
#include "src/common/cloudtree.h"

#include "src/modules/keypointsmodule.h"

namespace Ui {
class KeyPoint;
}

class KeyPoint : public QWidget
{
    Q_OBJECT

public:
    explicit KeyPoint(QWidget *parent = nullptr);
    ~KeyPoint();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;
    ProcessTree *processTree;


    void init();
    void add();
    void apply();
    void reset();

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::KeyPoint *ui;

    bool processEnable;
    KeyPointsModule keypoints;
};

#endif // KEYPOINT_H
