#ifndef FEATURE_H
#define FEATURE_H

#include <QWidget>
#include <QDesktopWidget>
#include <pcl/visualization/pcl_plotter.h>

#include "src/common/cloudtree.h"
#include "src/modules/features.h"

namespace Ui {
class Feature;
}

class Feature : public QWidget
{
    Q_OBJECT

public:
    explicit Feature(QWidget *parent = nullptr);
    ~Feature();
    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void preview();
    void apply();
    void reset();

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Feature *ui;
    Features feature;
    std::vector<FPFHFeature::Ptr,Eigen::aligned_allocator<FPFHFeature::Ptr>> fpfhs;
    std::vector<SHOTFeature::Ptr,Eigen::aligned_allocator<SHOTFeature::Ptr>> shots;
    PFHFeature::Ptr pfh;
    FPFHFeature::Ptr fpfh;
    VFHFeature::Ptr vfh;
    SHOTFeature::Ptr shot;
    pcl::visualization::PCLPlotter::Ptr plotter;
};

#endif // FEATURE_H
