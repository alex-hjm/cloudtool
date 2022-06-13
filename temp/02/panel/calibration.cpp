#include "calibration.h"
#include "ui_calibration.h"

Calibration::Calibration(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Calibration)
{
    ui->setupUi(this);
    connect(ui->tbtn_openfile,&QPushButton::clicked,this,&Calibration::openfile);
    connect(ui->btn_camcali,&QPushButton::clicked,this,&Calibration::camcali);
    connect(ui->btn_eyehandcali,&QPushButton::clicked,this,&Calibration::eyehandcali);
    QIntValidator* cw_Validator = new QIntValidator(0, 1000);
    QIntValidator* ch_Validator = new QIntValidator(0, 1000);
    QDoubleValidator* sw_Validator = new QDoubleValidator(0,1000,3);
    QDoubleValidator* sh_Validator = new QDoubleValidator(0,1000,3);
    ui->txt_corner_width->setValidator(cw_Validator);
    ui->txt_corner_height->setValidator(ch_Validator);
    ui->txt_size_width->setValidator(sw_Validator);
    ui->txt_size_height->setValidator(sh_Validator);

}

Calibration::~Calibration()
{
    delete ui;

}

void Calibration::init()
{

}

void Calibration::openfile()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Open pose file"));
    if(path.isEmpty())return;
    QFileInfo fileinfo(path);
    ui->txt_posefile->setText(fileinfo.fileName());
    QTextCodec *pcode = QTextCodec::codecForName("GB2312");
    std::string file_name=pcode->fromUnicode(fileinfo.filePath()).data();
    ifs.open(file_name, std::ios::in);
}

void Calibration::camcali()
{
    std::vector<Image> checkedImages=imageTree->getCheckedImages();
    int image_num=checkedImages.size();
    if(image_num<=0) {
        console->warning(tr("Please select some images!"));
        return;
    }
    console->info(tr("Image size : %1").arg(checkedImages.size()));
    cv::Size boardSize(ui->txt_corner_width->text().toInt(),ui->txt_corner_height->text().toInt());
    cv::Size2f squareSize(ui->txt_size_width->text().toFloat(),ui->txt_size_height->text().toFloat());
    if((boardSize.width <0)|(boardSize.height <0)){
        console->error(tr("Board size set Error!"));
        return ;
    }
    if((squareSize.width <0)|(squareSize.height <0)){
        console->error(tr("Square size set Error!"));
        return ;
    }
    cv::Size imageSize;
    std::vector<cv::Point2f> image_points_buff;
    std::vector<std::vector<cv::Point2f>> image_points_seq;
    console->info(tr("Start corner extraction..."));
    cv::waitKey(10);
    for (int i = 0; i < image_num; i++){
        cv::Mat image;
        cv::cvtColor(checkedImages[i].image, image, cv::COLOR_BGR2GRAY);
        if (i == 0){
            imageSize.width = image.cols;
            imageSize.height = image.rows;
            console->info((tr("Image size: %1 X %2").arg(image.cols).arg(image.rows)));
        }

        if (0 == cv::findChessboardCorners(image, boardSize, image_points_buff,
                                           cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE /*+ cv::CALIB_CB_FAST_CHECK*/)){
            console->warning(tr("Can not find chessboard corners! Please retry!"));
            return;
        }
        else{
            cv::cornerSubPix(image, image_points_buff, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            image_points_seq.push_back(image_points_buff);
            console->info((tr("Processing : %1 ").arg(i+1)+checkedImages[i].id.c_str()+" successfully!"));
            cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            cv::drawChessboardCorners(image, boardSize, image_points_buff, true);
            checkedImages[i].image=image;
            checkedImages[i].prefix("corner-");
            imageTree->insertImage(i,checkedImages[i],false);
            cv::waitKey(10);
        }
    }
    image_num=image_points_seq.size();
    console->info(tr("Corner extraction complete. Corner size: %1").arg(image_num));
    if(image_num<=4) {
        console->error(tr("Corner extraction error"));
        return ;
    }
    console->info(tr("Camera calibration start... "));
    cv::waitKey(10);
    std::vector<std::vector<cv::Point3f>> object_points;
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    std::vector<int> point_counts;
    cv::Mat dist_coeff = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));

    for (int i=0; i < image_num; i++){
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
        object_points.push_back(temp_point_set);
    }
    for (int i = 0; i < image_num; i++)
    {
        point_counts.push_back(boardSize.width * boardSize.height);
    }
    transVec.clear();
    rolatVec.clear();
    cv::calibrateCamera(object_points, image_points_seq, imageSize, camera_matrix, dist_coeff, rolatVec, transVec, 0);
    if((rolatVec.size()<=0)|(transVec.size()<=0)) {
        console->error(tr("Camera Calibration error"));
        return ;
    }
    console->info(tr("Calibration complete!"));
    Eigen::Matrix<float, 3, 3> cameraMatrix;
    Eigen::Matrix<float, 1, 5> distCoeff;
    cv::cv2eigen(camera_matrix,cameraMatrix);
    cv::cv2eigen(dist_coeff,distCoeff);
    ui->txt_output->clear();
    ui->txt_output->append(tr("Camera internal parameters:"));
    ui->txt_output->append(tool.QStringFromMatrix(cameraMatrix,6));
    ui->txt_output->append(tr("Camera distortion coefficient:"));
    ui->txt_output->append(tool.QStringFromMatrix(distCoeff,6));

    console->info(tr("Start to evaluate the calibration results."));
    double total_err = 0.0;
    double err = 0.0;
    std::vector<cv::Point2f> image_points2;
    console->info(tr("Calibration error of each image: "));
    for (int i = 0; i < image_num; i++) {
        std::vector<cv::Point3f> temp_point_set = object_points[i];
        cv::projectPoints(temp_point_set, rolatVec[i], transVec[i], camera_matrix, dist_coeff, image_points2);
        std::vector<cv::Point2f> temp_image_points = image_points_seq[i];
        cv::Mat temp_image_points_Mat = cv::Mat(1, temp_image_points.size(), CV_32FC2);
        cv::Mat temp_image_points2_Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
        for (int j = 0; j < temp_image_points.size(); j++){
            temp_image_points_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(temp_image_points[j].x, temp_image_points[j].y);
            temp_image_points2_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
        }
        err = cv::norm(temp_image_points_Mat, temp_image_points2_Mat, cv::NORM_L2);
        total_err += err /= point_counts[i];
        console->info(tr("The average error of image ")+checkedImages[i].id.c_str()+tr(" : %1 pixel.").arg(err));
    }
    console->info(tr("Total mean error : %1 pixel").arg(total_err / image_num));
    ui->txt_output->append(tr("Total mean error : %1 pixel").arg(total_err / image_num));
    console->info(tr("Evaluation completed!"));
}

void Calibration::eyehandcali()
{
    if (!ifs.is_open()) {
        console->warning(tr("Pose file not found"));
        return;
    }
    if((transVec.size()<=0)|(rolatVec.size()<=0)) {
        console->warning(tr("Please Calibrate Camera first"));
        return;
    }
    console->info(tr("Start eye hand calibration."));
    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    std::vector<cv::Mat> R_target2cam, t_target2cam;

    for (int i = 0; i < transVec.size(); i++) {
        //pose
        vector<double> pose;
        console->info(tr("Pose[ %1 ]:").arg(i+1));
        for (int j = 0; j < 6; j++) {
            double temp;
            ifs >> temp;
            pose.push_back(temp);
        }
        console->info(tr("XYZ: %1 %2 %3  RPY: %4 %5 %6") .arg(pose[0]).arg(pose[1]).arg(pose[2]).arg(pose[3]).arg(pose[4]).arg(pose[5]));
        Eigen::Matrix3f rotation_matrix=tool.RotMatrixFromEulerAngle(pose[3],pose[4],pose[5]);
        cv::Mat R;
        cv::eigen2cv(rotation_matrix,R);
        cv::Mat t = (cv::Mat_<float>(3,1) << pose[0], pose[1], pose[2]);
        R_gripper2base.push_back(R);
        t_gripper2base.push_back(t);

        console->info(tr("Cam[ %1 ]:").arg(i+1));
        cv::Mat cam_rot_temp;
        cv::Rodrigues(rolatVec.at(i), cam_rot_temp);
        R_target2cam.push_back(cam_rot_temp);
        t_target2cam.push_back(transVec.at(i));

        Eigen::Matrix3f rot_eigen;
        Eigen::Vector3f trans_eigen;
        cv::cv2eigen(cam_rot_temp, rot_eigen);
        cv::cv2eigen(transVec.at(i), trans_eigen);
        Eigen::Vector3f eulerAngle=rot_eigen.eulerAngles(0,1,2);
        console->info(tr("XYZ: %1 %2 %3  RPY: %4 %5 %6") .arg(trans_eigen[0]).arg(trans_eigen[1]).arg(trans_eigen[2])
                      .arg(eulerAngle(0)*180/M_PI).arg(eulerAngle(1)*180/M_PI).arg(eulerAngle(2)*180/M_PI));
    }
    ifs.close();

    cv::Mat R_cam2base_gripper, t_cam2base_gripper;
    int index=ui->cbox_calimethod->currentIndex();

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
        cv::calibrateHandEye(
                    R_base2gripper,  t_base2gripper,
                    R_target2cam,    t_target2cam,
                    R_cam2base_gripper,  t_cam2base_gripper,
                    cv::HandEyeCalibrationMethod(index));
        console->info(tr("Eye to hand calibration completed!"));
        ui->txt_output->append(tr("cam2base:"));


    } else if (ui->rbtn_eyeinhand->isChecked()) {
        //cam2tool
        cv::calibrateHandEye(
                    R_gripper2base,  t_gripper2base,
                    R_target2cam,    t_target2cam,
                    R_cam2base_gripper,  t_cam2base_gripper,
                    cv::HandEyeCalibrationMethod(index));
        console->info(tr("Eye in hand calibration completed!"));
        ui->txt_output->append(tr("cam2tool:"));

    }

    Eigen::Matrix3f rot_eigen;
    Eigen::Vector3f trans_eigen;
    cv::cv2eigen(R_cam2base_gripper, rot_eigen);
    cv::cv2eigen(t_cam2base_gripper, trans_eigen);

    Eigen::Matrix4f cam2base_gripper=tool.MatrixFromXYZRotMatrix(
                trans_eigen[0],trans_eigen[1],trans_eigen[2],rot_eigen);
    ui->txt_output->append(tool.QStringFromMatrix(cam2base_gripper.cast <float> (),6));
}



