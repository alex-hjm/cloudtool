#ifndef PROCESSPLAN_H
#define PROCESSPLAN_H

#include <QWidget>

#include "src/common/processtree.h"
#include "src/common/cloudtree.h"
#include "src/common/modeltree.h"

#include "src/modules/commonmodule.h"
#include "src/modules/featuremodule.h"
#include "src/modules/filtersmodule.h"
#include "src/modules/treemodule.h"
#include "src/modules/surfacemodule.h"
#include "src/modules/segmentationmodule.h"
#include "src/modules/keypointsmodule.h"

namespace Ui {
class ProcessPlan;
}

class ProcessPlan : public QWidget
{
    Q_OBJECT

public:
    explicit ProcessPlan(QWidget *parent = nullptr);
    ~ProcessPlan();

    Ui::ProcessPlan *ui;

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;
    ModelTree *modelTree;
    ProcessTree *processTree;

    void init();
    void start();
    void reset();
    void clear();

protected:
    void closeEvent(QCloseEvent *event);
    void processCloud(std::vector<Cloud> &cloud,std::vector<Process> &p,const int &i);

public slots:
   void processCaptureCloud(const CloudXYZRGBN::Ptr &);
   void captureStateChanged(bool,bool);

private:
    CommonModule common;
    FeatureModule feature;
    FiltersModule filters;
    SegmentationModule seg;
    SurfaceModule sur;
    TreeModule tree;

    int singleStep;

    ModelCoefs coefs;
    Cloud captureCloud;
    Model selectedModel;
    PolygonMesh::Ptr triangle;
    BoundingBox boundingbox;
    Eigen::Vector3f viewpoint;

    std::vector<Index> indexs;
    std::vector<Cloud> selectedClouds;
    std::vector<Cloud> allClouds;

    CloudXYZRGBN::Ptr devcie;
    std::vector<Index> processIndexs;
    std::vector<Process> allProcess;
    std::vector<Cloud> processedClouds;
    std::vector<CloudXYZRGBN::Ptr> segmentedClouds;

};

#endif // PROCESSPLAN_H
