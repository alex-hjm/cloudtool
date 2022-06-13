#ifndef NORMALS_H
#define NORMALS_H

#include <QDockWidget>
#include <QThread>
#include "common/cloudtree.h"
#include "modules/features.h"

namespace Ui {
class Normals;
}

class Normals : public QDockWidget
{
    Q_OBJECT

public:
    explicit Normals(QWidget *parent = nullptr);
    ~Normals();
    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void preview();
    void add();
    void apply();
    void reset();
signals:
    void normalEstimation(const Cloud::Ptr& cloud,int K,double R,const Eigen::Vector3f &viewpoint);
public slots:
    void result(const Cloud::Ptr &cloud,float time);
    void updateNormals();
protected:
    void closeEvent(QCloseEvent *event);
private:
    Ui::Normals *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    Features feature;
    QThread thread;
    std::vector<Cloud::Ptr> clouds_with_normals;
};

#endif // NORMALS_H
