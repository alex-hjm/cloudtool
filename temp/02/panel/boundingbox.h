#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <QWidget>
#include "src/common/cloudtree.h"
#include "src/modules/features.h"

namespace Ui {
class BoundingBox;
}

class BoundingBoxs : public QWidget
{
    Q_OBJECT
public:
    explicit BoundingBoxs(QWidget *parent = nullptr);
    ~BoundingBoxs();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void preview();
    void apply();
    void reset();

signals:
    void eulerAngles(float r,float p,float y);

public slots:
    void adjustbox(float r,float p,float y);

protected:
    void adjustEnable(bool state);
    void closeEvent(QCloseEvent *event);

private:
    Ui::BoundingBox *ui;

    int boxType;
    Features feature;
    std::vector<BoundingBox> boundingBoxs;
};

#endif // BOUNDINGBOX_H
