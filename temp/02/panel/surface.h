#ifndef SURFACE_H
#define SURFACE_H

#include <QWidget>
#include "src/common/cloudtree.h"
#include "src/common/modeltree.h"
#include "src/modules/surfaces.h"
namespace Ui {
class Surface;
}

class Surface : public QWidget
{
    Q_OBJECT

public:
    explicit Surface(QWidget *parent = nullptr);
    ~Surface();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;
    ModelTree *modelTree;

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
    Ui::Surface *ui;
    Surfaces sur;

    std::vector<PolygonMesh::Ptr> meshs;
};

#endif // SURFACE_H
