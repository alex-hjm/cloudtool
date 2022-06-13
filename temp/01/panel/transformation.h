#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <QDockWidget>
#include "common/cloudtree.h"
#include "common/tool.h"
namespace Ui {
class Transformation;
}

class Transformation : public QDockWidget
{
    Q_OBJECT

public:
    explicit Transformation(QWidget *parent = nullptr);
    ~Transformation();
    void init(Console* &co,CloudView* &cv,CloudTree* &ct);
    void preview();
    void add();
    void apply();
    void reset();

signals:
    void affine3f(const Eigen::Affine3f&);

public slots:
    void removeCloud(const std::string &id);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Transformation *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    Eigen::Affine3f affine;
};

#endif // TRANSFORMATION_H
