#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H

#include <QDockWidget>
#include <QDesktopWidget>
#include <pcl/visualization/pcl_plotter.h>
#include "common/cloudtree.h"
#include "modules/features.h"

namespace Ui {
class Descriptor;
}

class Descriptor : public QDockWidget
{
    Q_OBJECT

public:
    explicit Descriptor(QWidget *parent = nullptr);
    ~Descriptor();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void preview();
    void apply();
    void reset();

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Descriptor *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    PFHFeature::Ptr pfh;
    FPFHFeature::Ptr fpfh;
    VFHFeature::Ptr vfh;
    SHOTFeature::Ptr shot;
    pcl::visualization::PCLPlotter::Ptr plotter;
    std::vector<FPFHFeature::Ptr,Eigen::aligned_allocator<FPFHFeature::Ptr>> fpfhs;
    std::vector<SHOTFeature::Ptr,Eigen::aligned_allocator<SHOTFeature::Ptr>> shots;

};

#endif // DESCRIPTOR_H
