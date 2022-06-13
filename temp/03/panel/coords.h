#ifndef COORDS_H
#define COORDS_H

#include <QDialog>
#include "common/cloudtree.h"
#include "common/processtree.h"
#include "common/tool.h"
namespace Ui {
class Coords;
}

class Coords : public QDialog
{
    Q_OBJECT

public:
    explicit Coords(QWidget *parent = nullptr);
    ~Coords();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct,ProcessTree* pt);
    void addCoord();
    void closeCoord();
    void add();
    void apply();
    void reset();

public slots:
    void removeCoords(const std::string &id);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Coords *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    ProcessTree *process_tree;
};

#endif // COORDS_H
