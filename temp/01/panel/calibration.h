#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <QDockWidget>
#include <QThread>
#include <QDesktopServices>
#include <Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "common/imagetree.h"
#include "common/tool.h"


class Calibrate3D : public QObject
{
    Q_OBJECT
public:
    explicit Calibrate3D(QObject *parent = nullptr): QObject(parent){}

signals:
    void cameraResult(bool success,const std::vector<std::vector<cv::Point3f>> &objectPoints,const std::vector<std::vector<cv::Point2f>>&imagePoints,
                      const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs);
    void handEyeResult(const cv::Mat &R_cam2gripper, const cv::Mat &t_cam2gripper);
public slots:
    void calibrateCamera(const std::vector<std::vector<cv::Point3f>> &objectPoints,const std::vector<std::vector<cv::Point2f>>&imagePoints,
                         const cv::Size &imageSize)
    {
        std::vector<cv::Mat> tvecs;std::vector<cv::Mat> rvecs;
        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
        cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
        if(0==cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0))
            emit cameraResult(false,objectPoints,imagePoints,cameraMatrix,distCoeffs,rvecs,tvecs);
        else
            emit cameraResult(true,objectPoints,imagePoints,cameraMatrix,distCoeffs,rvecs,tvecs);

    }

    void calibrateExternelParameter(const std::vector<std::vector<cv::Point3f>> &objectPoints,const std::vector<std::vector<cv::Point2f>>&imagePoints,
                         const cv::Size &imageSize)
    {
        std::vector<cv::Mat> tvecs;std::vector<cv::Mat> rvecs;
        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
        cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
        if(0==cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0))
            emit cameraResult(false,objectPoints,imagePoints,cameraMatrix,distCoeffs,rvecs,tvecs);
        else
            emit cameraResult(true,objectPoints,imagePoints,cameraMatrix,distCoeffs,rvecs,tvecs);

    }

    void calibrateHandEye(const std::vector<cv::Mat> &R_gripper2base, const std::vector<cv::Mat> &t_gripper2base,
                          const std::vector<cv::Mat> &R_target2cam, const std::vector<cv::Mat> &t_target2cam,int method)
    {
        cv::Mat R_cam2gripper, t_cam2gripper;
        cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper,
                                   cv::HandEyeCalibrationMethod(method));
        emit handEyeResult(R_cam2gripper,t_cam2gripper);
    }
};

namespace Ui {
class Calibration;
}

class Calibration : public QDockWidget
{
    Q_OBJECT

public:
    explicit Calibration(QWidget *parent = nullptr);
    ~Calibration();

    void init(Console* &co,ImageView* &iv,ImageTree* &it);
    void loadPoseFile();
    void loadInternelParameterFile();
    void caculateExternelParameter();
    void calibrateEyeHand();
    void saveResult();
signals:
    void calibrateCamera(const std::vector<std::vector<cv::Point3f>> &objectPoints,const std::vector<std::vector<cv::Point2f>>&imagePoints,
                         const cv::Size &imageSize);

    void calibrateExternelParameter(const std::vector<std::vector<cv::Point3f>> &objectPoints,const std::vector<std::vector<cv::Point2f>>&imagePoints,
                         const cv::Size &imageSize);
    void calibrateHandEye(const std::vector<cv::Mat> &R_gripper2base, const std::vector<cv::Mat> &t_gripper2base,
                          const std::vector<cv::Mat> &R_target2cam, const std::vector<cv::Mat> &t_target2cam,int method);
public slots:
    void cameraResult(bool success,const std::vector<std::vector<cv::Point3f>> &objectPoints,const std::vector<std::vector<cv::Point2f>>&imagePoints,
                      const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs);
    void handEyeResult(const cv::Mat &R_cam2gripper, const cv::Mat &t_cam2gripper);
private:
    Ui::Calibration *ui;
    Console *console;
    ImageView *image_view;
    ImageTree *image_tree;
    QThread thread;
    std::string pose_file_name;
    std::string cam_file_name;
    std::vector<cv::Mat> trans_vec;
    std::vector<cv::Mat> rolat_vec;
};



#endif // CALIBRATION_H
