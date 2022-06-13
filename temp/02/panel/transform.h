#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <QWidget>
#include "src/common/cloudtree.h"
#include "src/common/tool.h"

namespace Ui {
class Transform;
}

class Transform : public QWidget
{
    Q_OBJECT

public:
    explicit Transform(QWidget *parent = nullptr);
    ~Transform();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void add();
    void apply();
    void reset();

signals:
    void Affine3f(Eigen::Affine3f);

public slots:
    void transCloud(Eigen::Affine3f);
    void removeCloud(const string &id);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Transform *ui;
    Tool tool;

    Eigen::Affine3f affine;
};

#endif // TRANSFORM_H
