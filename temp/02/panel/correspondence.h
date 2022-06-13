#ifndef CORRESPONDENCE_H
#define CORRESPONDENCE_H

#include <QWidget>
#include "src/common/cloudtree.h"
#include "src/modules/registrations.h"
namespace Ui {
class Correspondence;
}

class Correspondence : public QWidget
{
    Q_OBJECT

public:
    explicit Correspondence(QWidget *parent = nullptr);
    ~Correspondence();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;
    void init();
    void setTarget();
    void setSource();
    void preview();
    void apply();
    void reset();

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::Correspondence *ui;
    CorrespondencesPtr all_correspondence;
    CorrespondencesPtr remain_correspondences;
    Registrations reg;
    Cloud targetCloud;
    Cloud sourceCloud;
};

#endif // CORRESPONDENCE_H
