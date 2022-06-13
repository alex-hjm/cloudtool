#ifndef SEGMENTATIONS_H
#define SEGMENTATIONS_H

#include <QDockWidget>
#include <QThread>
#include "common/cloudtree.h"
#include "modules/segmentation.h"

namespace Ui {
class Segmentations;
}

class Segmentations : public QDockWidget
{
    Q_OBJECT

public:
    explicit Segmentations(QWidget *parent = nullptr);
    ~Segmentations();
     void init(Console* &co,CloudView* &cv,CloudTree* &ct);
     void preview();
     void add();
     void apply();
     void reset();
signals:
     void PlaneSACSegmentation(const Cloud::Ptr &cloud,double distance_weightconst,double threshold,nt max_iterations,bool negative);
     void EuclideanClusterExtraction(const Cloud::Ptr &cloud,double tolerance,int min_cluster_size,int max_cluster_size);
     void RegionGrowing(const Cloud::Ptr &cloud,int min_cluster_size,int max_cluster_size,bool smoothMode,bool CurvatureTest,
                        float SmoothnessThreshold,float CurvatureThreshold,int NumberOfNeighbours);
public slots:
     void result(const std::vector<Cloud::Ptr> &cloud,float time);
protected:
    void closeEvent(QCloseEvent *event);
private:
    Ui::Segmentations *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    QThread thread;
    std::vector<Cloud::Ptr> segmented_clouds;
};

#endif // SEGMENTATIONS_H
