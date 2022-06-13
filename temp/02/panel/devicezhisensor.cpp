#include "devicezhisensor.h"
#include "ui_Devicezhisensor.h"

DeviceZhiSensor::DeviceZhiSensor(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DeviceZhiSensor),
    capturedCloud(new CloudXYZRGBN),
    isConnected(false),
    isCapturing(false),
    outputType(0),
    pointgigestream(NULL),
    rgbgigestream(NULL),
    graygigestream(NULL)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&DeviceZhiSensor::add);
    connect(ui->btn_search,&QPushButton::clicked,this,&DeviceZhiSensor::searchDevice);
    connect(ui->btn_connect,&QPushButton::clicked,this,&DeviceZhiSensor::connectDevice);
    connect(ui->btn_capture,&QPushButton::clicked,this,&DeviceZhiSensor::captureDevice);
}

DeviceZhiSensor::~DeviceZhiSensor()
{
    delete ui;
}

void DeviceZhiSensor::init()
{
    connect(ui->cbox_output,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        outputType=index;
        if(index==0)
            ui->slider_refresh->setMinimum(500);
        else
            ui->slider_refresh->setMinimum(10);
    });

    connect(ui->slider_refresh,&QSlider::valueChanged,[=](int value){
        float time=double(value)/1000;
        QString str=QString::number(time,'f',3)+" s";
        ui->txt_freshtime->setText(str);
        if(isCapturing){
            captureTimer->stop();
            captureTimer->start(value);
        }
    });

    connect(ui->check_autofresh,&QCheckBox::stateChanged,[=](int state)
    {
        if(isConnected)
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
        if(isConnected)
            if(state) {
                ui->spin_ex_time->setEnabled(false);
                if((outputType==0)|(outputType==1)) {
                    camera.SetAutoExposure(0,1);
                } else {
                    camera.SetAutoExposure(0,0);
                }
            }
            else {
                ui->spin_ex_time->setEnabled(true);
                if((outputType==0)|(outputType==1)) {
                    camera.SetAutoExposure(1,1);
                    camera.SetExposureTime(ui->spin_ex_time->value(),1);
                } else {
                    camera.SetAutoExposure(1,0);
                    camera.SetExposureTime(ui->spin_ex_time->value(),0);
                }
            }
    });

    connect(ui->spin_ex_time,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value){
        if(isConnected)
            if((outputType==0)|(outputType==1)) {
                camera.SetExposureTime(value,1);
            } else {
                camera.SetExposureTime(value,0);
            }
    });

    connect(ui->spin_mu_ex,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value){
        if(isConnected)
            if((outputType==0)|(outputType==1)) {
                camera.SetMutipleExposure(value);
            } else {
                camera.SetMutipleExposure(value);
            }
    });

    captureTimer=new QTimer;
    connect(captureTimer,&QTimer::timeout,[=]{
        time.tic();
        switch (outputType) {
        case 0://pointcloud
            if(pointgigestream->TimeoutCapture(point_data, 3000000)!=0) {
                console->warning(tr("Capture pointcloud stream failed."));
                return;
            }
            if(rgbgigestream->TimeoutCapture(rgb_data, 3000000)!=0) {
                console->warning(tr("Capture rgb stream failed."));
                return;
            }
            this->captureCloud();
            break;
        case 1://rgb
            if(rgbgigestream->TimeoutCapture(rgb_data, 3000000)!=0) {
                console->warning(tr("Capture rgb stream failed."));
                return;
            }
            this->captureImage(outputType);
            break;
        case 2://gray
            if(graygigestream->TimeoutCapture(gray_data, 3000000)!=0) {
                console->warning(tr("Capture gray stream failed."));
                return;
            }
            this->captureImage(outputType);
            break;
        }
        cloudView->showInfoText(std::to_string(time.toc())+" ms",30,"info");
    });
}

void DeviceZhiSensor::searchDevice()
{
    if(!isConnected){
        std::vector<DiscoveryInfo>().swap(discovery_info);
        discovery.SetLogLevelSwitch(1, 0, 0, 1);
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

void DeviceZhiSensor::connectDevice()
{
    if(!isConnected){
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

        point_data = new PhotoInfo;
        point_data->pixel = new char[width * height * 6];
        memset(point_data->pixel, 0, width * height * 6);
        gray_data = new PhotoInfo;
        gray_data->pixel = new char[width * height];
        memset(gray_data->pixel, 0, width * height);
        rgb_data = new PhotoInfo;
        rgb_data->pixel = new char[width_rgb * height_rgb * 3];
        memset(rgb_data->pixel, 0, width_rgb * height_rgb * 3);
        pointcloud =new float[width * height * 6];
        memset(pointcloud, 0, width * height * 6);
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
            if((outputType==0)|(outputType==1)) {
                camera.SetAutoExposure(0,1);
            } else {
                camera.SetAutoExposure(0,0);
            }
        } else {
            if((outputType==0)|(outputType==1)) {
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
        isConnected=true;
    }
    else {
        if(isCapturing){
            isCapturing=false;
            cloudView->removeShape("info");
            captureTimer->stop();
            ui->btn_capture->setText(tr("Capture"));
            ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        }
        if(cloudView->contains("zhisensor"))
            cloudView->removeCloud("zhisensor");
        cloudView->removeShape("info");
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
        isConnected=false;

    }
}

void DeviceZhiSensor::captureDevice()
{
    if(isConnected){
        if(!ui->check_autofresh->isChecked()) {
            camera.SetTriggerCount(1);
            camera.SetRGBTriggerCount(1);
            console->info(tr("Capturing..."));
            switch (outputType) {
            case 0://pointcloud
                if(pointgigestream->TimeoutCapture(point_data, 3000000)!=0) {
                    console->warning(tr("Capture pointcloud stream failed."));
                    return;
                }
                if(rgbgigestream->TimeoutCapture(rgb_data, 3000000)!=0) {
                    console->warning(tr("Capture rgb stream failed."));
                    return;
                }
                this->captureCloud();
                break;
            case 1://rgb
                if(rgbgigestream->TimeoutCapture(rgb_data, 3000000)!=0) {
                    console->warning(tr("Capture rgb stream failed."));
                    return;
                }
                this->captureImage(outputType);
                break;
            case 2://gray
                if(graygigestream->TimeoutCapture(gray_data, 3000000)!=0) {
                    console->warning(tr("Capture gray stream failed."));
                    return;
                }
                this->captureImage(outputType);
                break;
            }
        } else {
            if(!isCapturing){
                console->info(tr("Capturing..."));
                captureTimer->start(ui->slider_refresh->value());
                isCapturing=true;
                ui->btn_capture->setText(tr("Stop"));
                ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
            } else {
                isCapturing=false;
                captureTimer->stop();
                cloudView->removeShape("info");
                ui->btn_capture->setText(tr("Capture"));
                ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/start.svg"));
            }
        }
    } else {
        console->warning(tr("Please connect a device."));
        return;
    }
}

void DeviceZhiSensor::add()
{
    QFileInfo fileinfo("../ZhiSensor/zhisensor-"+QDateTime::currentDateTime().toString("hh-mm-ss"));
    if(outputType==0){
        if(capturedCloud->empty()) {
            console->warning(tr("Please capture a pointcloud."));
            return;
        }
        Cloud cloud(capturedCloud,fileinfo);
        cloudView->removeCloud("zhisensor");
        cloudTree->insertCloud(-1,cloud,true);
        cloudView->updateBoundingBox(cloud.box,cloud.boxid);
    } else if(outputType==1) {
        if(cv_rgbImage.empty()){
            console->warning(tr("Please capture a color data."));
            return;
        }
        Image image(cv_rgbImage,fileinfo);
        imageTree->insertImage(-1,image,true);
    }else if(outputType==2) {
        if(cv_grayImage.empty()){
            console->warning(tr("Please capture a depth data."));
            return;
        }
        Image image(cv_grayImage,fileinfo);
        imageTree->insertImage(-1,image,true);
    }
}

void DeviceZhiSensor::captureCloud()
{
    capturedCloud.reset(new CloudXYZRGBN);
    camera.FusionImageTo3D(*rgb_data, *point_data, pointcloud);
    for (int i = 0; i <(point_data->pixel_height * point_data->pixel_width); ++i){
        PointXYZRGBN point;
        point.x = *(pointcloud + 6 * i + 0);
        point.y = *(pointcloud + 6 * i + 1);
        point.z = *(pointcloud + 6 * i + 2);
        if(ui->check_withoutcolor->isChecked()) {
            point.b = *(pointcloud + 6 * i + 3);
            point.g = *(pointcloud + 6 * i + 4);
            point.r = *(pointcloud + 6 * i + 5);
            if (point.b == 0 && point.g == 0 && point.r == 0){
                continue;
            }
        }
        if (point.z == 0){
            continue;
        }
        capturedCloud->points.push_back(point);
    }
    cloudView->updateCloud(capturedCloud,"zhisensor");
}

void DeviceZhiSensor::captureImage(int index)
{
    if(index==1) { //rgb
        camera.RawdataToRgb888(*rgb_data);
        QImage image((const uchar*)rgb_data->pixel, rgb_data->pixel_width, rgb_data->pixel_height, QImage::Format_RGB888);
        cv_rgbImage=graphicsView->QImage2Mat(image,true);
        graphicsView->addImage(cv_rgbImage);
    } else if(index==2)  {//gray
        cv_grayImage=cv::Mat(gray_data->pixel_height,gray_data->pixel_width,CV_8UC1,gray_data->pixel);
        graphicsView->addImage(cv_grayImage);
    }
}

void DeviceZhiSensor::closeEvent(QCloseEvent *event)
{
    captureTimer->stop();
    graphicsView->removeImage();
    cloudView->removeCloud("zhisensor");
    cloudView->removeShape("info");
    delete[] point_data->pixel;
    delete point_data;
    delete[] gray_data->pixel;
    delete gray_data;
    delete[] rgb_data->pixel;
    delete rgb_data;
    delete[] pointcloud;
    if(isConnected) {
        camera.StreamOff(0, graygigestream);
        camera.StreamOff(1, pointgigestream);
        camera.StreamOff(2, rgbgigestream);
        camera.CameraDisconnect();
    }
    return QWidget::closeEvent(event);
}


