#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <QWidget>

#include "src/common/cloudtree.h"
#include "src/common/tool.h"
#include "src/modules/registrations.h"

namespace Ui {

class Registration;
}

class Registration : public QWidget
{
    Q_OBJECT

public:
    explicit Registration(QWidget *parent = nullptr);
    ~Registration();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void setTarget();
    void setSource();
    void preview();
    void add();
    void apply();
    void reset();

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Registration *ui;
    Tool tool;
    int num_iterations;
    QFileInfo fileInfo;
    Registrations reg;
    Index targetIndex;
    Cloud targetCloud;
    Index sourceIndex;
    Cloud sourceCloud;
    CloudXYZRGBN::Ptr ail_cloud;
    CorrespondencesPtr all_corr;
};

#endif // REGISTRATION_H
