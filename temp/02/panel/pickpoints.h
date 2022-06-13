#ifndef PICKPOINTS_H
#define PICKPOINTS_H

#include <QDialog>
#include "src/common/cloudtree.h"
#include "src/common/processtree.h"
#include "src/common/tool.h"

namespace Ui {
class PickPoints;
}

class PickPoints : public QDialog
{
    Q_OBJECT

public:
    explicit PickPoints(QWidget *parent = nullptr);
    ~PickPoints();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void add();
    void del();
    void start();
    void reset();

public slots:
    void pickPoints(int);
    void pickArea(std::vector<int>);
    void reverseCloud(bool);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::PickPoints *ui;
    FeatureModule feature;
    Tool tool;

    bool isPicking;
    Index index;
    Cloud selectedCloud;
    CloudXYZRGBN::Ptr pickCloud;
    std::vector<int> indices;
    std::vector<Cloud> allClouds;
};

#endif // PICKPOINTS_H
