#ifndef FILTER_H
#define FILTER_H

#include <QWidget>

#include "src/common/cloudtree.h"
#include "src/modules/filters.h"

namespace Ui {
class Filter;
}

class Filter : public QWidget
{
    Q_OBJECT

public:
    explicit Filter(QWidget *parent = nullptr);
    ~Filter();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void preview();
    void add();
    void apply();
    void reset();

public slots:
    void removeCloud(const string &id);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Filter *ui;
    Filters filters;
    std::vector<CloudXYZRGBN::Ptr> filteredClouds;
};

#endif // FILTER_H
