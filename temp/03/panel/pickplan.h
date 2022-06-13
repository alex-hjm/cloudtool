#ifndef PICKPLAN_H
#define PICKPLAN_H

#include <QDockWidget>
#include "common/cloudtree.h"
#include "common/processtree.h"
#include "common/pathtable.h"
#include "common/tool.h"
#include "panel/Registrations.h"

#include "modules/filter.h"
namespace Ui {
class PickPlan;
}

class PickPlan : public QDockWidget
{
    Q_OBJECT

public:
    explicit PickPlan(QWidget *parent = nullptr);
    ~PickPlan();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct,ProcessTree *pt,PathTable *ptb);
    void setTemplate();
    void setTarget();
    void preview();
    void reset();
    void addFinalPath();
    void openEyeHandFile();
    void openPath();
    void add();
    void start();
    void autoRun();


public slots:
    void processCloud(Cloud::Ptr &target_cloud);
    void restart();

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::PickPlan *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    ProcessTree *process_tree;
    PathTable *path_table;
    int target_index;
    int path_index;
    std::vector<int> final_path_index;

    bool is_start;
    bool is_auto_run;
    Cloud::Ptr target_cloud;
    std::vector<Cloud::Ptr> template_cloud;
    Registration::Result final_result;
    Cloud::Ptr ail_cloud;

    Eigen::Affine3f  eye_hand_affine;
    std::vector<Eigen::Affine3f> path_points;

};

#endif // PICKPLAN_H
