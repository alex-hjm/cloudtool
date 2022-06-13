#ifndef COORDINATE_H
#define COORDINATE_H

#include <QDialog>
#include "src/common/cloudtree.h"

namespace Ui {
class Coordinate;
}

class Coordinate : public QDialog
{
    Q_OBJECT

public:
    explicit Coordinate(QWidget *parent = nullptr);
    ~Coordinate();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void add();
    void apply();
    void reset();

public slots:
    void removeCloud(const string &id);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Coordinate *ui;
    std::vector<Eigen::Affine3f> affines;
};

#endif // COORDINATE_H
