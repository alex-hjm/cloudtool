#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <QWidget>
#include <QThread>
#include <iostream>
#include <string>

#include <Eigen/Core>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>


#include "src/common/imagetree.h"
#include "src/common/tool.h"

namespace Ui {
class Calibration;
}

class Calibration : public QWidget
{
    Q_OBJECT

public:
    explicit Calibration(QWidget *parent = nullptr);
    ~Calibration();
     Console *console;
     ImageTree *imageTree;
     GraphicsView *graphicsView;

     void init();
     void openfile();
     void camcali();
     void eyehandcali();

private:
    Ui::Calibration *ui;
    Tool tool;

    std::ifstream ifs;
    std::vector<cv::Mat> transVec;
    std::vector<cv::Mat> rolatVec;
};

#endif // CALIBRATION_H
