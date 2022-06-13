#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <QWidget>
#include "src/common/cloudtree.h"
#include "src/modules/segmentations.h"

namespace Ui {
class Segmentation;
}

class Segmentation : public QWidget
{
    Q_OBJECT

public:
    explicit Segmentation(QWidget *parent = nullptr);
    ~Segmentation();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void preview();
    void add();
    void apply();
    void reset();

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Segmentation *ui;

    Segmentations seg;
    pcl::SacModel modelType;
    ModelCoefs coefs;
    std::vector<CloudXYZRGBN::Ptr> segmentedClouds;

};

#endif // SEGMENTATION_H
