#include "devicezhisensor.h"
#include "ui_devicezhisensor.h"

DeviceZhisensor::DeviceZhisensor(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::DeviceZhisensor),captured_cloud(new Cloud),
    is_connected(false),is_capturing(false), output_type(0),
    cv_rgbImage(new Image),cv_grayImage(new Image)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&DeviceZhisensor::add);
    connect(ui->btn_search,&QPushButton::clicked,this,&DeviceZhisensor::searchDevice);
    connect(ui->btn_connect,&QPushButton::clicked,this,&DeviceZhisensor::connectDevice);
    connect(ui->btn_capture,&QPushButton::clicked,this,&DeviceZhisensor::captureDevice);
    connect(ui->btn_getcaminter,&QPushButton::clicked,this,&DeviceZhisensor::getCamIntrinsics);
}

DeviceZhisensor::~DeviceZhisensor()
{
    delete ui;
}

void DeviceZhisensor::init(Console* &co,CloudView* &cv,CloudTree* &ct,ImageTree *&it,ImageView *&iv,ProcessTree* pt)
{
    console=co;cloud_view=cv;cloud_tree=ct;image_tree=it;image_view=iv;process_tree=pt;
    connect(process_tree,&ProcessTree::startCapture,this,&DeviceZhisensor::captureDevice);
    connect(ui->cbox_output,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        output_type=index;
        if(index==0)
            ui->slider_refresh->setMinimum(500);
        else
            ui->slider_refresh->setMinimum(10);
    });
    connect(ui->slider_refresh,&QSlider::valueChanged,[=](int value){
        float time=double(value)/1000;
        QString str=QString::number(time,'f',3)+" s";
        ui->txt_freshtime->setText(str);
        if(is_capturing){
            capture_timer->stop();
            capture_timer->start(value);
        }
    });
    connect(ui->check_autofresh,&QCheckBox::stateChanged,[=](int state)
    {
        if(is_connected)
            if(state) {
                camera.SetRGBTriggerMode(0);
                camera.SetTriggerMode(0);
            } else {
                camera.SetRGBTriggerMode(1);
                camera.SetTriggerMode(1);
            }
    });
    connect(ui->check_exposure_time,&QCheckBox::stateChanged,[=](int state)
    {
        if(state)
            ui->spin_ex_time->setEnabled(false);
        else
            ui->spin_ex_time->setEnabled(true);
        if(is_connected)
            if(state) {
                if(output_type==0||output_type==1) {
                    camera.SetAutoExposure(0,1);
                } else {
                    camera.SetAutoExposure(0,0);
                }
            } else {
                if(output_type==0||output_type==1) {
                    camera.SetAutoExposure(1,1);
                    camera.SetExposureTime(ui->spin_ex_time->value(),1);
                } else {
                    camera.SetAutoExposure(1,0);
                    camera.SetExposureTime(ui->spin_ex_time->value(),0);
                }
            }
    });
    connect(ui->spin_ex_time,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value){
        if(is_connected)
            if(output_type==0||output_type==1)
                camera.SetExposureTime(value,1);
            else
                camera.SetExposureTime(value,0);
    });
    connect(ui->spin_mu_ex,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value){
        if(is_connected)
            if(output_type==0||output_type==1) {
                camera.SetMutipleExposure(value);
            } else {
                camera.SetMutipleExposure(value);
            }
    });
    capture_timer=new QTimer;
    connect(capture_timer,&QTimer::timeout,[=]{this->output();});
}

void DeviceZhisensor::searchDevice()
{
    if(!is_connected){
        Discovery discovery;
        std::vector<DiscoveryInfo>().swap(discovery_info);
        discovery.SetLogLevelSwitch(1, 0, 0, 0);
        int device_count = discovery.DiscoverCamera(&discovery_info);
        if(device_count<=0){
            console->warning(tr("Found 0 connected device."));
            return ;
        }
        console->info(tr("Found %1 connected device.").arg(device_count));
        int camer_sort = discovery.CameraSort(&discovery_info, 1);
        if(camer_sort!=0){
            console->error(tr("The device sorting failed."));
            return ;
        }
        ui->cbox_ip->clear();
        for (int i = 0; i < device_count; i++) {
            QString IPAddress=discovery.ConvertIpIntToString(discovery_info[i].camera_ip);
            ui->cbox_ip->addItem(IPAddress);
            console->info(tr("Num %1 device IP address :").arg(i+1)+IPAddress);
        }
    } else{
        console->info(tr("The device is connected"));
        return;
    }
}

void DeviceZhisensor::connectDevice()
{
    if(!is_connected){
        if(ui->cbox_ip->count()<=0){
            console->warning(tr("No device,please search the device first."));
            return ;
        }
        int num=ui->cbox_ip->currentIndex();
        int connect = camera.CameraConnect(&discovery_info[num]);
        if(connect!=0) {
            console->error(tr("Connect the device failed."));
            return ;
        }
        //init  data//////////////////////////////////////////////////////////////////
        unsigned int width_register_addr = 0;
        unsigned int height_register_addr = 0;
        unsigned int width_register_addr_RGB = 0;
        unsigned int height_register_addr_RGB = 0;
        width_register_addr = camera.GetRegisterAddr("Width");
        height_register_addr = camera.GetRegisterAddr("Height");
        camera.ReadRegister(width_register_addr, (int *)&width);
        camera.ReadRegister(height_register_addr, (int *)&height);
        width_register_addr_RGB = camera.GetRegisterAddr("Width") + 0x100;
        height_register_addr_RGB = camera.GetRegisterAddr("Height") + 0x100;
        camera.ReadRegister(width_register_addr_RGB, (int *)&width_rgb);
        camera.ReadRegister(height_register_addr_RGB, (int *)&height_rgb);
        ///TriggerMode/////////////////////////////////////////////////////////
        if(ui->check_autofresh->isChecked()) {
            camera.SetRGBTriggerMode(0);
            camera.SetTriggerMode(0);
        } else {
            camera.SetRGBTriggerMode(1);
            camera.SetTriggerMode(1);
        }
        ///AcquisitionStart////////////////////////////////////////////////////
        camera.StreamOn(0, &graygigestream);
        camera.StreamOn(1, &pointgigestream);
        camera.StreamOn(2, &rgbgigestream);
        camera.AcquisitionStart();
        pointgigestream->FlushBuffer();
        graygigestream->FlushBuffer();
        rgbgigestream->FlushBuffer();
        ///AutoExposure////////////////////////////////////////////////////////
        if(ui->check_exposure_time->isChecked()) {
            if((output_type==0)|(output_type==1)) {
                camera.SetAutoExposure(0,1);
            } else {
                camera.SetAutoExposure(0,0);
            }
        } else {
            if((output_type==0)|(output_type==1)) {
                camera.SetAutoExposure(1,1);
                camera.SetExposureTime(ui->spin_ex_time->value(),1);
            } else {
                camera.SetAutoExposure(1,0);
                camera.SetExposureTime(ui->spin_ex_time->value(),0);
            }
        }
        ///////////////////////////////////////////////////////////////////////
        console->info(tr("Connect the device sucessfully!"));
        ui->btn_connect->setText(tr("Discon"));
        ui->btn_connect->setIcon(QIcon(":/icon/resource/icon/disconnect.svg"));
        is_connected=true;
    }
    else {
        if(is_capturing){
            is_capturing=false;
            capture_timer->stop();
            ui->btn_capture->setText(tr("Capture"));
            ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        }
        if(cloud_view->contains("zhisensor"))
            cloud_view->removeCloud("zhisensor");
        camera.StreamOff(0, graygigestream);
        camera.StreamOff(1, pointgigestream);
        camera.StreamOff(2, rgbgigestream);
        int disconnect = camera.CameraDisconnect();
        if(disconnect!=0) {
            console->error(tr("Disconnect the device failed."));
            return ;
        }
        console->info(tr("Disconnect the device sucessfully!"));
        ui->btn_connect->setText(tr("Connect"));
        ui->btn_connect->setIcon(QIcon(":/icon/resource/icon/connect.svg"));
        is_connected=false;
    }
}

void DeviceZhisensor::captureDevice()
{
    if(is_connected){
        if(!ui->check_autofresh->isChecked()) {
            console->showStatusMessage("capturing...",0);
            this->output();
            console->clearStatusMessage();
        } else {
            if(!is_capturing){
                console->showStatusMessage("capturing...",0);
                capture_timer->start(ui->slider_refresh->value());
                is_capturing=true;
                ui->btn_capture->setText(tr("Stop"));
                ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
            } else {
                is_capturing=false;
                capture_timer->stop();
                console->clearStatusMessage();
                ui->btn_capture->setText(tr("Capture"));
                ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/start.svg"));
            }
        }
    } else {
        console->warning(tr("Please connect a device."));
        return;
    }
}

void DeviceZhisensor::getCamIntrinsics()
{
    if(is_connected){
        float Kc[5],K[9];
        if(output_type!=2)
            camera.GetCamInternelParameter(1,Kc,K);
        else
            camera.GetCamInternelParameter(0,Kc,K);
        Eigen::Matrix<float, 3, 3> camera_matrix;
        Eigen::Matrix<float, 1, 5> dist_coeffs;
        for (int r = 0, idx = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                camera_matrix(r,c) = K[idx++];
        for (int c = 0; c < 5; c++)
            dist_coeffs(0,c) = Kc[c];
        cv::Mat cameraMatrix,distCoeffs;
        cv::eigen2cv(camera_matrix, cameraMatrix);
        cv::eigen2cv(dist_coeffs, distCoeffs);
        QString path= QFileDialog::getSaveFileName(this, tr ("Save camera intrinsics"),"", "xml(*.xml)");
        if(!path.endsWith("xml"))path.append(".xml");
        std::string file_name=path.toLocal8Bit().toStdString();
        cv::FileStorage inCailFs(file_name.c_str(), cv::FileStorage::WRITE);
        inCailFs << "camera_matrix" << cameraMatrix;
        inCailFs << "dist_coffes" << distCoeffs;
        inCailFs.release();
        console->info(tr("Save camera intrinsics to ")+path+(" sucessfully!"));
    } else {
        console->warning(tr("Please connect a device."));
        return;
    }

}

void DeviceZhisensor::output()
{
    PhotoInfo* point_data = new PhotoInfo;
    point_data->pixel = new char[width * height * 6];
    memset(point_data->pixel, 0, width * height * 6);
    PhotoInfo* gray_data = new PhotoInfo;
    gray_data->pixel = new char[width * height];
    memset(gray_data->pixel, 0, width * height);
    PhotoInfo* rgb_data = new PhotoInfo;
    rgb_data->pixel = new char[width_rgb * height_rgb * 3];
    memset(rgb_data->pixel, 0, width_rgb * height_rgb * 3);
    float* pointcloud =new float[width * height * 6];
    memset(pointcloud, 0, width * height * 6);
    if(!ui->check_autofresh->isChecked()) {
        camera.SetTriggerCount(1);
        camera.SetRGBTriggerCount(1);
    }
    if(output_type==0) {
        if(pointgigestream->TimeoutCapture(point_data, 3000000)!=0) {
            console->warning(tr("Capture pointcloud stream failed."));
            return;
        }
        if(rgbgigestream->TimeoutCapture(rgb_data, 3000000)!=0) {
            console->warning(tr("Capture rgb stream failed."));
            return;
        }
        camera.FusionImageTo3D(*rgb_data, *point_data, pointcloud);
        captured_cloud.reset(new Cloud);
        //captured_cloud->reserve(point_data->pixel_height *point_data->pixel_width);
        for (int i = 0; i <(point_data->pixel_height * point_data->pixel_width); ++i){
            PointXYZRGBN point;
            point.x = *(pointcloud + 6 * i + 0);
            point.y = *(pointcloud + 6 * i + 1);
            point.z = *(pointcloud + 6 * i + 2);
            point.b = *(pointcloud + 6 * i + 3);
            point.g = *(pointcloud + 6 * i + 4);
            point.r = *(pointcloud + 6 * i + 5);
            if (point.b == 0 && point.g == 0 && point.r == 0){
                continue;
            }
            if (point.z == 0){
                continue;
            }
            captured_cloud->points.push_back(point);
        }
        captured_cloud->setId("zhisensor");
        cloud_view->addCloud(captured_cloud,captured_cloud->id);
        if(ui->check_withoutcolor->isChecked())
            cloud_view->setCloudColor(captured_cloud,captured_cloud->id,"z");
        if(process_tree->enable())
            emit process_tree->captureCloud(captured_cloud);
    } else if(output_type==1) {
        if(rgbgigestream->TimeoutCapture(rgb_data, 3000000)!=0) {
            console->warning(tr("Capture rgb stream failed."));
            return;
        }
        camera.RawdataToRgb888(*rgb_data);
        QImage qimage((const uchar*)rgb_data->pixel, rgb_data->pixel_width, rgb_data->pixel_height, QImage::Format_RGB888);
        cv_rgbImage->copy(image_view->QImage2Mat(qimage,true));
        image_view->addImage(cv_rgbImage,false);
    } else if(output_type==2) {
        if(graygigestream->TimeoutCapture(gray_data, 3000000)!=0) {
            console->warning(tr("Capture gray stream failed."));
            return;
        }
        cv_grayImage->copy(cv::Mat(gray_data->pixel_height,gray_data->pixel_width,CV_8UC1,gray_data->pixel));
        image_view->addImage(cv_grayImage,false);
    }
    delete[] point_data->pixel;
    delete point_data;
    delete[] gray_data->pixel;
    delete gray_data;
    delete[] rgb_data->pixel;
    delete rgb_data;
    delete[] pointcloud;
}

void DeviceZhisensor::add()
{
    QFileInfo info("../ZhiSensor/zhisensor-"+QDateTime::currentDateTime().toString("hh-mm-ss"));
    switch (output_type)
    {
    case 0:
        if(captured_cloud->empty()) {
            console->warning(tr("Please capture a pointcloud."));
            return;
        }
        captured_cloud->setInfo(info);
        captured_cloud->update();
        cloud_view->removeCloud("zhisensor");
        cloud_tree->insertCloud(-1,captured_cloud,true);
        cloud_view->updateCube(captured_cloud->box,captured_cloud->box_id);
        break;
    case 1:
        if(cv_rgbImage->empty()){
            console->warning(tr("Please capture a color data."));
            return;
        }
        cv_rgbImage->setInfo(info);
        image_tree->insertImage(-1,cv_rgbImage,true);
        break;
    case 2:
        if(cv_grayImage->empty()){
            console->warning(tr("Please capture a depth data."));
            return;
        }
        cv_grayImage->setInfo(info);
        image_tree->insertImage(-1,cv_grayImage,true);
        break;
    }
    console->info(tr("Added successfully!"));
}

void DeviceZhisensor::closeEvent(QCloseEvent *event)
{
    capture_timer->stop();
    image_view->removeImage();
    cloud_view->removeCloud("zhisensor");
    if(is_connected) {
        camera.StreamOff(0, graygigestream);
        camera.StreamOff(1, pointgigestream);
        camera.StreamOff(2, rgbgigestream);
        camera.CameraDisconnect();
    }
    return QWidget::closeEvent(event);
}


