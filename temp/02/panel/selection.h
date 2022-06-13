#ifndef SELECTION_H
#define SELECTION_H

#include <QDialog>
#include "src/common/cloudtree.h"
#include "src/common/processtree.h"
#include "src/modules/commonmodule.h"

namespace Ui {
class Selection;
}

class Selection : public QDialog
{
    Q_OBJECT

public:
    explicit Selection(QWidget *parent = nullptr);
    ~Selection();

    Console *console;
    CloudView *cloudView;
    CloudTree*cloudTree;
    ProcessTree *processTree;

    void init();
    void preview();
    void apply();
    void reset();

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Selection *ui;
    CommonModule com;
    std::vector<CloudXYZRGBN::Ptr> finalClouds;
    std::vector<Cloud> selectedClouds;
};

#endif // SELECTION_H
