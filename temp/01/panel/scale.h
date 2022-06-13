#ifndef SCALE_H
#define SCALE_H

#include <QDialog>
#include "common/cloudtree.h"
#include "modules/common.h"

namespace Ui {
class Scale;
}

class Scale : public QDialog
{
    Q_OBJECT

public:
    explicit Scale(QWidget *parent = nullptr);
    ~Scale();
    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void add();
    void apply();
    void reset();

signals:
    void scale(double x,double y,double z);

public slots:
    void scaleCloud(double x,double y,double z);
    void removeCloud(const std::string &id);

protected:
    void closeEvent(QCloseEvent *event);
private:
    Ui::Scale *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    std::vector<Cloud::Ptr> scaledClouds;
};

#endif // SCALE_H
