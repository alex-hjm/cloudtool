#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <QDockWidget>
#include "common/cloudtree.h"
#include "modules/features.h"
namespace Ui {
class BoundingBox;
}

class BoundingBox : public QDockWidget
{
    Q_OBJECT

public:
    explicit BoundingBox(QWidget *parent = nullptr);
    ~BoundingBox();
    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void preview();
    void apply();
    void reset();

signals:
    void eulerAngles(float r,float p,float y);

public slots:
    void adjustEnable(bool state);
    void adjustBox(float r,float p,float y);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::BoundingBox *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    int box_type;
    std::vector<Box> boundingBoxs;
};

#endif // BOUNDINGBOX_H
