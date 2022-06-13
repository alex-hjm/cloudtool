#ifndef NORMALS_H
#define NORMALS_H

#include <QWidget>
#include "src/common/cloudtree.h"
#include "src/modules/features.h"

namespace Ui {
class Normals;
}

class Normals : public QWidget
{
    Q_OBJECT

public:
    explicit Normals(QWidget *parent = nullptr);
    ~Normals();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void preview();
    void add();
    void apply();
    void reset();

protected:
    void showNormals();
    void closeEvent(QCloseEvent *event);

private:
    Ui::Normals *ui;
    Features feature;

    Eigen::Vector3f viewpoint;
    std::vector<CloudXYZRGBN::Ptr> cloudsWithNormals;
};

#endif // NORMALS_H
