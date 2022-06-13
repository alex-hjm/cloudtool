#ifndef FILTERS_H
#define FILTERS_H

#include <QDockWidget>
#include <QThread>
#include "common/cloudtree.h"
#include "common/processtree.h"
#include "modules/filter.h"

namespace Ui {
class Filters;
}

class Filters : public QDockWidget
{
    Q_OBJECT

public:
    explicit Filters(QWidget *parent = nullptr);
    ~Filters();
    void init(Console* &co,CloudView* &cv,CloudTree* &ct,ProcessTree* pt);
    void preview();
    void add();
    void apply();
    void reset();
public slots:
    void getRangeFromCloud(int index);
    void removeCloud(const std::string &id);
protected:
    void closeEvent(QCloseEvent *event);
private:
    Ui::Filters *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    ProcessTree *process_tree;
    std::vector<Cloud::Ptr> filtered_clouds;
};

#endif // FILTERS_H
