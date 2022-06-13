#ifndef REGISTRATIONS_H
#define REGISTRATIONS_H

#include <QDockWidget>
#include "common/cloudtree.h"
#include "common/tool.h"
#include "common/processtree.h"
#include "modules/registration.h"
#include "modules/features.h"
#include "panel/template.h"

namespace Ui {
class Registrations;
}

class Registrations : public QDockWidget
{
    Q_OBJECT

public:
    explicit Registrations(QWidget *parent = nullptr);
    ~Registrations();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct,ProcessTree* pt);
    void setTemplate();
    void setTarget();
    void preview();
    void add();
    void apply();
    void reset();

protected:
    void closeEvent(QCloseEvent *event);
private:
    Ui::Registrations *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    ProcessTree *process_tree;
    Cloud::Ptr target_cloud;
    std::vector<Cloud::Ptr> template_cloud;
    Registration::Result final_result;
    Cloud::Ptr ail_cloud;
};

#endif // REGISTRATIONS_H
