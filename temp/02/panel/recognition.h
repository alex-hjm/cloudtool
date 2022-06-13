#ifndef RECOGNITION_H
#define RECOGNITION_H

#include <QWidget>

#include "src/common/cloudtree.h"
#include "src/common/tool.h"
#include "src/modules/recognitions.h"

namespace Ui {
class Recognition;
}

class Recognition : public QWidget
{
    Q_OBJECT

public:
    explicit Recognition(QWidget *parent = nullptr);
    ~Recognition();
    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

    void init();
    void setModel();
    void setScene();
    void preview();
    void add();
    void apply();
    void reset();

private:
    Ui::Recognition *ui;
    Tool tool;
    Recognitions rec;
    Index modelIndex;
    Cloud modelCloud;
    Index sceneIndex;
    Cloud sceneCloud;
    Clusters clusters;

    std::vector<CloudXYZRGBN::Ptr> recClouds;

};

#endif // RECOGNITION_H
