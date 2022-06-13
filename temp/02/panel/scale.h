#ifndef SCALE_H
#define SCALE_H

#include <QDialog>
#include "src/common/cloudtree.h"
#include "src/modules/common.h"

namespace Ui {
class Scale;
}

class Scale : public QDialog
{
    Q_OBJECT

public:
    explicit Scale(QWidget *parent = nullptr);
    ~Scale();

    Console *console;
    CloudView *cloudView;
    CloudTree*cloudTree;

    void init();
    void add();
    void apply();
    void reset();

signals:
    void scale(double x,double y,double z);

public slots:
    void scaleCloud(double x,double y,double z);
    void removeCloud(const string &id);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Scale *ui;
    Common com;
    std::vector<CloudXYZRGBN::Ptr> scaledClouds;

};

#endif // SCALE_H
