#include "calibration.h"
#include "ui_calibration.h"

Calibration::Calibration(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::Calibration),pose_file_name(""),cam_file_name("")
{
    ui->setupUi(this);
    connect(ui->tbtn_openfile,&QPushButton::clicked,this,&Calibration::loadPoseFile);
    connect(ui->tbtn_opencampara,&QPushButton::clicked,this,&Calibration::loadInternelParameterFile);
    connect(ui->btn_externelparameter,&QPushButton::clicked,this,&Calibration::caculateExternelParameter);
    connect(ui->btn_eyehandcali,&QPushButton::clicked,this,&Calibration::calibrateEyeHand);
    connect(ui->btn_save,&QPushButton::clicked,this,&Calibration::saveResult);
    QIntValidator* cw_Validator = new QIntValidator(0, 1000);
    QIntValidator* ch_Validator = new QIntValidator(0, 1000);
    QDoubleValidator* sw_Validator = new QDoubleValidator(0,1000,3);
    QDoubleValidator* sh_Validator = new QDoubleValidator(0,1000,3);
    ui->txt_corner_width->setValidator(cw_Validator);
    ui->txt_corner_height->setValidator(ch_Validator);
    ui->txt_size_width->setValidator(sw_Validator);
    ui->txt_size_height->setValidator(sh_Validator);
    Calibrate3D *calib3D=new Calibrate3D;
    calib3D->moveToThread(&thread);
    qRegisterMetaType<cv::Size>("cv::Size");
    qRegisterMetaType<cv::Size>("cv::Size &");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<cv::Mat>("cv::Mat &");
    qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
    qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat> &");
    qRegisterMetaType<std::vector<std::vector<cv::Point3f>>>("std::vector<std::vector<cv::Point3f>>");
    qRegisterMetaType<std::vector<std::vector<cv::Point3f>>>("std::vector<std::vector<cv::Point3f>> &");
    qRegisterMetaType<std::vector<std::vector<cv::Point2f>>>("std::vector<std::vector<cv::Point2f>>");
    qRegisterMetaType<std::vector<std::vector<cv::Point2f>>>("std::vector<std::vector<cv::Point2f>> &");
    connect(&thread,&QThread::finished,calib3D,&QObject::deleteLater);
    connect(this,&Calibration::calibrateCamera,calib3D,&Calibrate3D::calibrateCamera);
    connect(this,&Calibration::calibrateHandEye,calib3D,&Calibrate3D::calibrateHandEye);
    connect(calib3D,&Calibrate3D::cameraResult,this,&Calibration::cameraResult);
    connect(calib3D,&Calibrate3D::handEyeResult,this,&Calibration::handEyeResult);
    thread.start();
    connect(ui->rbtn_calibrate,&QRadioButton::clicked,[=](bool checked)
    {
        if(checked){
            ui->txt_cam_inter->setEnabled(false);
            ui->tbtn_opencampara->setEnabled(false);
        }
    });
    connect(ui->rbtn_loadinterparas,&QRadioButton::clicked,[=](bool checked)
    {
        if(checked){
            ui->txt_cam_inter->setEnabled(true);
            ui->tbtn_opencampara->setEnabled(true);
        }
    });

}

Calibration::~Calibration()
{
    thread.quit();
    thread.wait();
    delete ui;
}
void Calibration::init(Console* &co,ImageView* &iv,ImageTree* &it)
{
    console=co;image_view=iv;image_tree=it;
}

void Calibration::loadPoseFile()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Open pose file"));
    if(path.isEmpty())return;
    QFileInfo fileinfo(path);
    ui->txt_posefile->setText(fileinfo.fileName());
    //QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    pose_file_name=path.toLocal8Bit().toStdString();
    console->info(tr("Load robot pose file :")+path+tr(" successfully!"));
}

void Calibration::loadInternelParameterFile()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Open internel parameter file"));
    if(path.isEmpty())return;
    ui->txt_cam_inter->setText(path);
    //QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    cam_file_name=path.toLocal8Bit().toStdString();
    console->info(tr("Load internel parameter file :")+path+tr(" successfully!"));
}

void Calibration::caculateExternelParameter()
{
    cv::Size boardSize(ui->txt_corner_width->text().toInt(),ui->txt_corner_height->text().toInt());
    cv::Size2f squareSize(ui->txt_size_width->text().toFloat(),ui->txt_size_height->text().toFloat());
    if(boardSize.width <0||boardSize.height <0){
        console->error(tr("Board size set Error!"));
        return ;
    }
    if(squareSize.width <0||squareSize.height <0){
        console->error(tr("Square size set Error!"));
        return ;
    }
    std::vector<Image::Ptr> checkedImages=image_tree->getCheckedImages();
    int image_num=checkedImages.size();
    if(image_num<=0) {
        console->warning(tr("Please select some images!"));
        return;
    }
    console->info(tr("Image size : %1").arg(image_num));
    cv::Size imageSize;
    imageSize.width = checkedImages.front()->cols;
    imageSize.height = checkedImages.front()->rows;
    console->info(tr("Image resolution: %1 X %2").arg(imageSize.width).arg(imageSize.height));
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<int> point_counts;
    std::vector<cv::Point3f> temp_point_set;
    for (int j = 0; j < boardSize.height; j++){
        for (int k = 0; k < boardSize.width; k++){
            cv::Point3f real_point;
            real_point.x = k * squareSize.width;
            real_point.y = j * squareSize.height;
            real_point.z = 0;
            temp_point_set.push_back(real_point);
        }
    }
    for (int i=0; i < image_num; i++){
        object_points.push_back(temp_point_set);
        point_counts.push_back(boardSize.width * boardSize.height);
    }
    std::vector<cv::Point2f> image_points_buff;
    std::vector<std::vector<cv::Point2f>> image_points_seq;
    if(ui->rbtn_calibrate->isChecked()) {//calibrate camera
        console->info(tr("Start corner extraction..."));
        for (int i = 0; i < image_num; i++){
            Image::Ptr copy_image=checkedImages[i]->makeShared();
            cv::cvtColor(*copy_image, *copy_image, cv::COLOR_BGR2GRAY);
            if (0 == cv::findChessboardCorners(*copy_image, boardSize, image_points_buff,
                                               cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE /*+ cv::CALIB_CB_FAST_CHECK*/)){
                console->error(tr("Can not find chessboard corners! Please retry!"));
                return;
            }
            cv::cornerSubPix(*copy_image, image_points_buff, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            image_points_seq.push_back(image_points_buff);
            cv::cvtColor(*copy_image, *copy_image, cv::COLOR_GRAY2BGR);
            cv::drawChessboardCorners(*copy_image, boardSize, image_points_buff, true);
            copy_image->prefix("corner-");
            image_tree->insertImage(i,copy_image,true);
            console->info((tr("Processing : %1 ").arg(i+1)+checkedImages[i]->id.c_str()+" successfully!"));
            cv::waitKey(10);
        }
        console->info(tr("Corner extraction complete. Corner size: %1").arg(image_points_seq.size()));
        if(image_num!=image_points_seq.size())return;
        console->info(tr("Camera calibration start... "));
        console->showStatusMessage("caculating...",0);
        console->showProgressBar();
        emit calibrateCamera(object_points, image_points_seq, imageSize);
    } else if(ui->rbtn_loadinterparas->isChecked()) {
        if(ui->txt_cam_inter->text().isEmpty()) {
            console->warning(tr("Please load camera internel parameter first!"));
            return;
        }
        cv::FileStorage fs(cam_file_name.c_str(), cv::FileStorage::READ);
        cv::Mat cameraMatrix, distCoeffs;
        fs["camera_matrix"] >> cameraMatrix;
        fs["dist_coffes"] >> distCoeffs;
        if(cameraMatrix.empty() || distCoeffs.empty()){
            console->warning(tr("Load camera intrinsic failed!"));
            return;
        }
        trans_vec.clear();
        rolat_vec.clear();
        for (int i = 0; i < image_num; i++){
            Image::Ptr copy_image=checkedImages[i]->makeShared();
            cv::cvtColor(*copy_image, *copy_image, cv::COLOR_BGR2GRAY);
            if (0 == cv::findChessboardCorners(*copy_image, boardSize, image_points_buff,cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE /*+ cv::CALIB_CB_FAST_CHECK*/)){
                console->error(tr("Can not find chessboard corners! Please retry!"));
                return;
            }
            cv::cornerSubPix(*copy_image, image_points_buff, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            image_points_seq.push_back(image_points_buff);
            cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
            cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
            cv::solvePnP(temp_point_set, image_points_buff, cameraMatrix, distCoeffs, rvec, tvec);
            trans_vec.push_back(tvec);
            rolat_vec.push_back(rvec);
            cv::cvtColor(*copy_image, *copy_image, cv::COLOR_GRAY2BGR);
            cv::drawChessboardCorners(*copy_image, boardSize, image_points_buff, true);
            copy_image->prefix("corner-");
            image_tree->insertImage(i,copy_image,true);
            console->info((tr("Processing : %1 ").arg(i+1)+checkedImages[i]->id.c_str()+" successfully!"));
            cv::waitKey(10);
        }
        console->showStatusMessage("caculating...",0);
        console->showProgressBar();
        this->cameraResult(true,object_points,image_points_seq,cameraMatrix,distCoeffs,rolat_vec,trans_vec);
    }
}

void Calibration::calibrateEyeHand()
{
    if(ui->txt_posefile->text().isEmpty()) {
        console->warning(tr("Please load camera internel parameter first!"));
        return;
    }
    std::ifstream ifs;
    ifs.open(pose_file_name, std::ios::in);
    if (!ifs.is_open()) {
        console->warning(tr("Pose file open failed"));
        return;
    }
    if(trans_vec.size()<=0||rolat_vec.size()<=0) {
        console->warning(tr("Please calibrate camera first!"));
        return;
    }
    console->info(tr("Start eye hand calibration."));
    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    std::vector<cv::Mat> R_target2cam, t_target2cam;
    for (int i = 0; i < trans_vec.size(); i++) {
        std::vector<double> pose;
        for (int j = 0; j < 6; j++) {
            double temp;
            ifs >> temp;
            pose.push_back(temp);
        }
        console->info(tr("-----------------------------------------------------------------------------"));
        console->info(tr("Pose[ %1 ]: XYZ: %2 %3 %4  RPY: %5 %6 %7").arg(i+1).arg(pose[0]).arg(pose[1]).arg(pose[2]).arg(pose[3]).arg(pose[4]).arg(pose[5]));
        Eigen::Matrix3f rotation_matrix=Tool::RotMatrixFromEulerAngle(pose[3],pose[4],pose[5]);
        cv::Mat R;
        cv::eigen2cv(rotation_matrix,R);
        cv::Mat t = (cv::Mat_<float>(3,1) << pose[0], pose[1], pose[2]);
        R_gripper2base.push_back(R);
        t_gripper2base.push_back(t);

        cv::Mat cam_rot_temp;
        cv::Rodrigues(rolat_vec.at(i), cam_rot_temp);
        R_target2cam.push_back(cam_rot_temp);
        t_target2cam.push_back(trans_vec.at(i));

        Eigen::Matrix3f rot_eigen;
        Eigen::Vector3f trans_eigen;
        cv::cv2eigen(cam_rot_temp, rot_eigen);
        cv::cv2eigen(trans_vec.at(i), trans_eigen);
        Eigen::Vector3f eulerAngle=rot_eigen.eulerAngles(0,1,2);
        console->info(tr("Cam [ %1 ]: XYZ: %2 %3 %4  RPY: %5 %6 %7").arg(i+1).arg(trans_eigen[0]).arg(trans_eigen[1]).arg(trans_eigen[2])
                .arg(eulerAngle(0)*180/M_PI).arg(eulerAngle(1)*180/M_PI).arg(eulerAngle(2)*180/M_PI));
        console->info(tr("-----------------------------------------------------------------------------"));
    }
    ifs.close();
    ui->txt_output->clear();
    if(ui->rbtn_eyetohand->isChecked()) {
        std::vector<cv::Mat> R_base2gripper, t_base2gripper;
        unsigned long size = R_gripper2base.size();
        R_base2gripper.reserve(size);
        t_base2gripper.reserve(size);
        for (size_t i = 0; i < size; i++) {
            cv::Mat R = R_gripper2base[i];
            cv::Mat Rt = R.t();
            R_base2gripper.push_back(Rt);
            cv::Mat t = t_gripper2base[i];
            cv::Mat tinv = -Rt * t;
            t_base2gripper.push_back(tinv);
        }
        //cam2base
        console->showStatusMessage("caculating...",0);
        console->showProgressBar();
        emit calibrateHandEye(R_base2gripper,t_base2gripper,R_target2cam,t_target2cam,ui->cbox_calimethod->currentIndex());
    } else if (ui->rbtn_eyeinhand->isChecked()) {
        //cam2tool
        console->showStatusMessage("caculating...",0);
        console->showProgressBar();
        emit calibrateHandEye(R_gripper2base,t_gripper2base,R_target2cam,t_target2cam,ui->cbox_calimethod->currentIndex());
    }

}

void Calibration::saveResult()
{
    if(ui->txt_output->toPlainText().isEmpty())return;
    QString path =QFileDialog::getSaveFileName(this, tr ("Save Result"),"", "txt(*.txt)");
    if(path.isEmpty())return;
    if (!path.endsWith (".txt", Qt::CaseInsensitive))
        path.append(".txt");
    QFile file(path);
    if (!file.open(QFile::WriteOnly | QFile::Text))
    {
        console->error(tr("Write result to ")+path+(" failed!"));
        return;
    }
    QTextStream out(&file);
    out << ui->txt_output->toPlainText();
    file.close();
    QDesktopServices::openUrl(QUrl(QUrl::fromLocalFile(path)));
    console->info(tr("Write result to ")+path+(" sucessfully!"));
}

void Calibration::cameraResult(bool success,const std::vector<std::vector<cv::Point3f>> &objectPoints,const std::vector<std::vector<cv::Point2f>>&imagePoints,
                               const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs)
{
    if(!success) {
        console->error(tr("Camera Calibration failed"));
        return ;
    } else {
        if(ui->rbtn_calibrate->isChecked()) {
            trans_vec=tvecs;rolat_vec=rvecs;
            console->info(tr("Camera Calibration completed!"));
        }
        Eigen::Matrix<float, 3, 3> camera_matrix;
        Eigen::Matrix<float, 1, 5> dist_coeffs;
        cv::cv2eigen(cameraMatrix,camera_matrix);
        cv::cv2eigen(distCoeffs,dist_coeffs);
        ui->txt_output->clear();
        ui->txt_output->append(tr("Camera internal parameters:"));
        ui->txt_output->append(Tool::QStringFromMatrix(camera_matrix,6));
        ui->txt_output->append(tr("Camera distortion coefficient:"));
        ui->txt_output->append(Tool::QStringFromMatrix(dist_coeffs,6));

        console->info(tr("Start to evaluate the calibration results."));
        cv::Size boardSize(ui->txt_corner_width->text().toInt(),ui->txt_corner_height->text().toInt());
        int image_num=objectPoints.size();
        std::vector<int> point_counts;
        for (int i=0; i < image_num; i++){
            point_counts.push_back(boardSize.width * boardSize.height);
        }
        double total_err = 0.0;
        double err = 0.0;
        std::vector<cv::Point2f> image_points2;
        console->info(tr("Calibration error of each image: "));
        for (int i = 0; i < image_num; i++) {
            std::vector<cv::Point3f> temp_point_set = objectPoints[i];
            cv::projectPoints(temp_point_set, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, image_points2);
            std::vector<cv::Point2f> temp_image_points = imagePoints[i];
            cv::Mat temp_image_points_Mat = cv::Mat(1, temp_image_points.size(), CV_32FC2);
            cv::Mat temp_image_points2_Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
            for (int j = 0; j < temp_image_points.size(); j++){
                temp_image_points_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(temp_image_points[j].x, temp_image_points[j].y);
                temp_image_points2_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
            }
            err = cv::norm(temp_image_points_Mat, temp_image_points2_Mat, cv::NORM_L2);
            total_err += err /= point_counts[i];
            console->info(tr("The average error of  No. %1 image : %2 pixel.").arg(i+1).arg(err));
        }
        console->info(tr("Total mean error : %1 pixel").arg(total_err / image_num));
        ui->txt_output->append(tr("Total mean error : %1 pixel").arg(total_err / image_num));
        console->info(tr("Calibration Evaluation completed!"));
    }
    console->clearStatusMessage();
    console->closeProgressBar();
}

void Calibration::handEyeResult(const cv::Mat &R_cam2gripper, const cv::Mat &t_cam2gripper)
{
    console->clearStatusMessage();
    console->closeProgressBar();
    if(ui->rbtn_eyetohand->isChecked()) {
        //cam2base
        console->info(tr("Eye to hand calibration completed!"));
        ui->txt_output->append(tr("cam2base:"));
    } else if (ui->rbtn_eyeinhand->isChecked()) {
        //cam2tool
        console->info(tr("Eye in hand calibration completed!"));
        ui->txt_output->append(tr("cam2tool:"));
    }
    Eigen::Matrix3f rot_eigen;
    Eigen::Vector3f trans_eigen;
    cv::cv2eigen(R_cam2gripper, rot_eigen);
    cv::cv2eigen(t_cam2gripper, trans_eigen);
    Eigen::Matrix4f cam2base_gripper=Tool::MatrixFromXYZRotMatrix(
                trans_eigen[0],trans_eigen[1],trans_eigen[2],rot_eigen);
    ui->txt_output->append(Tool::QStringFromMatrix(cam2base_gripper.cast <float> (),6));

}
