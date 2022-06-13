#ifndef TREESEARCH_H
#define TREESEARCH_H

#include <QDockWidget>
#include "common/cloudtree.h"
#include "modules/tree.h"

namespace Ui {
class TreeSearch;
}

class TreeSearch : public QDockWidget
{
    Q_OBJECT

public:
    explicit TreeSearch(QWidget *parent = nullptr);
    ~TreeSearch();
    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void pick();
    void add();
    void apply();
    void reset();

public slots:
    void pickPoint(const Point2D&);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::TreeSearch *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    Cloud::Ptr search_cloud;
    bool is_picking;
};

#endif // TREESEARCH_H
