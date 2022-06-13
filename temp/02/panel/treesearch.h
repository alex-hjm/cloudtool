#ifndef TREESEARCH_H
#define TREESEARCH_H

#include <QWidget>
#include "src/common/cloudtree.h"
#include "src/modules/tree.h"

namespace Ui {
class TreeSearch;
}

class TreeSearch : public QWidget
{
    Q_OBJECT

public:
    explicit TreeSearch(QWidget *parent = nullptr);
    ~TreeSearch();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void pick();
    void add();
    void apply();
    void reset();

public slots:
    void leftPressPoint(Point2D);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::TreeSearch *ui;
    Tree tree;
    bool isPicking;
    CloudXYZRGBN::Ptr searchCloud;
};

#endif // TREESEARCH_H
