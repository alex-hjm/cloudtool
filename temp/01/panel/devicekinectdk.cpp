#include "devicekinectdk.h"
#include "ui_devicekinectdk.h"

DeviceKinectDK::DeviceKinectDK(QWidget *parent) :
    QDockWidget(parent),captured_cloud(new Cloud),cv_rgbImage(new Image),cv_depth(new Image),cv_irImage(new Image),
    ui(new Ui::DeviceKinectDK),is_connected(false),is_capturing(false),
    output_type(0),config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&DeviceKinectDK::add);
    connect(ui->btn_connect,&QPushButton::clicked,this,&DeviceKinectDK::connectDevice);
    connect(ui->btn_capture,&QPushButton::clicked,this,&DeviceKinectDK::captureDevice);
}

DeviceKinectDK::~DeviceKinectDK()
{
    delete ui;
}

void DeviceKinectDK::init(Console* &co,CloudView* &cv,CloudTree* &ct,ImageTree *&it,ImageView *&iv)
{
    console=co;cloud_view=cv;cloud_tree=ct;image_tree=it;image_view=iv;
    connect(ui->cbox_depth,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        config.depth_mode=k4a_depth_mode_t(index+1);
        if(index==3){
            if(ui->rbtn_30fps->isChecked()){
                ui->rbtn_15fps->setChecked(true);
                config.camera_fps=K4A_FRAMES_PER_SECOND_15;
            }
            ui->rbtn_30fps->setEnabled(false);
        }
        else
            ui->rbtn_30fps->setEnabled(true);
    });
    connect(ui->cbox_colorformat,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        config.color_format=k4a_image_format_t(index);
        if(index==1||index==2){
            ui->cbox_resolution->setCurrentIndex(0);
            ui->cbox_resolution->setEnabled(false);
            config.color_resolution=k4a_color_resolution_t(1);
        }
        else
            ui->cbox_resolution->setEnabled(true);
    });
    connect(ui->cbox_resolution,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        config.color_resolution=k4a_color_resolution_t(index+1);
        if(index==5){
            if(ui->rbtn_30fps->isChecked()){
                ui->rbtn_15fps->setChecked(true);
                config.camera_fps=K4A_FRAMES_PER_SECOND_15;
            }
            ui->rbtn_30fps->setEnabled(false);
        }
        else
            ui->rbtn_30fps->setEnabled(true);
    });
    connect(ui->cbox_output,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        output_type=index;
        if(index==0)
            ui->slider_refreshtime->setMinimum(30);
        else
            ui->slider_refreshtime->setMinimum(1);
    });
    connect(ui->rbtn_5fps,&QRadioButton::clicked,[=](bool state){
        if(state)config.camera_fps=K4A_FRAMES_PER_SECOND_5;
    });
    connect(ui->rbtn_15fps,&QRadioButton::clicked,[=](bool state){
        if(state)config.camera_fps=K4A_FRAMES_PER_SECOND_15;
    });
    connect(ui->rbtn_30fps,&QRadioButton::clicked,[=](bool state){
        if(state)config.camera_fps=K4A_FRAMES_PER_SECOND_30;
    });
    connect(ui->slider_refreshtime,&QSlider::valueChanged,[=](int value){
        float time=double(value)/1000;
        QString str=QString::number(time,'f',3)+" s";
        ui->txt_refreshtime->setText(str);
        if(is_capturing){
            capture_timer->stop();
            capture_timer->start(value);
        }
    });
    capture_timer=new QTimer(this);
    connect(capture_timer,&QTimer::timeout,[=]{this->output();});
    ui->cbox_depth->setCurrentIndex(0);
    ui->cbox_resolution->setCurrentIndex(1);
    ui->cbox_colorformat->setCurrentIndex(3);
    config.depth_mode=K4A_DEPTH_MODE_NFOV_2X2BINNED;
    config.color_resolution=K4A_COLOR_RESOLUTION_1080P;
    config.color_format=K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.camera_fps=K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;
}

void DeviceKinectDK::connectDevice()
{
    if(!is_connected){
        try{
            int device_count = k4a::device::get_installed_count();
            if(device_count<=0){
                console->warning(tr("Found 0 connected device."));
                return ;
            }
            console->info(tr("Found %1 connected device.").arg(device_count));
            device=k4a::device::open(K4A_DEVICE_DEFAULT);
            device.start_cameras(&config);
            calibration = device.get_calibration(config.depth_mode, config.color_resolution);
            transformation = k4a::transformation(calibration);
            console->info(tr("Open the device sucessfully!"));
            ui->btn_connect->setText(tr("Discon"));
            ui->btn_connect->setIcon(QIcon(":/icon/resource/icon/disconnect.svg"));
            is_connected=true;
        } catch (k4a::error error){
            console->error(error.what());
            is_connected=false;
            return;
        }
    } else {
        if(is_capturing){
            is_capturing=false;
            capture_timer->stop();
            ui->btn_capture->setText(tr("Capture"));
            ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        }
        if(cloud_view->contains("k4a"))
            cloud_view->removeCloud("k4a");
        try{
            if(device)
                device.close();
            transformation.destroy();
            console->info(tr("Close the device sucessfully!"));
            ui->btn_connect->setText(tr("Connect"));
            ui->btn_connect->setIcon(QIcon(":/icon/resource/icon/connect.svg"));
            is_connected=false;
        }
        catch (k4a::error error){
            console->error(error.what());
            is_connected=false;
            return;
        }
    }
}

void DeviceKinectDK::captureDevice()
{
    if(is_connected&&device){
        if(!ui->check_autofresh->isChecked()){
            console->showStatusMessage("capturing...",0);
            this->output();
            console->clearStatusMessage();
        } else {
            if(!is_capturing){
                console->showStatusMessage("capturing...",0);
                capture_timer->start(ui->slider_refreshtime->value());
                is_capturing=true;
                ui->btn_capture->setText(tr("Stop"));
                ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
            }
            else{
                is_capturing=false;
                capture_timer->stop();
                console->clearStatusMessage();
                ui->btn_capture->setText(tr("Capture"));
                ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/start.svg"));
            }
        }
    }
    else{
        console->warning(tr("Please connect a device."));
        return;
    }
}

void DeviceKinectDK::add()
{
    QFileInfo info("../KinectDK/k4a-"+QDateTime::currentDateTime().toString("hh-mm-ss"));
    switch (output_type)
    {
    case 0:
        if(captured_cloud->empty()) {
            console->warning(tr("Please capture a pointcloud."));
            return;
        }
        captured_cloud->setInfo(info);
        captured_cloud->update();
        cloud_view->removeCloud("k4a");
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
        if(cv_depth->empty()){
            console->warning(tr("Please capture a depth data."));
            return;
        }
        cv_depth->setInfo(info);
        image_tree->insertImage(-1,cv_depth,true);
        break;
    case 3:
        if(cv_irImage->empty()){
            console->warning(tr("Please capture a infrared data."));
            return;
        }
        cv_irImage->setInfo(info);
        image_tree->insertImage(-1,cv_irImage,true);
        break;
    }
    console->info(tr("Added successfully!"));
}

void DeviceKinectDK::output()
{
    try {
        if (!device.get_capture(&capture, std::chrono::milliseconds(500))) return;
        k4a::image depth_image = capture.get_depth_image();
        k4a::image color_image = capture.get_color_image();
        k4a::image infrared_image=capture.get_ir_image();
        if(depth_image==NULL||color_image==NULL||infrared_image==NULL){
            console->error(tr("Failed to capture,retry!"));
            return;
        }
        if(output_type==0) {
            captured_cloud->clear();
            int color_image_width_pixels = color_image.get_width_pixels();
            int color_image_height_pixels = color_image.get_height_pixels();

            k4a::image transformed_depth_image = NULL;
            transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                                                         color_image_width_pixels,
                                                         color_image_height_pixels,
                                                         color_image_width_pixels * (int)sizeof(uint16_t));
            k4a::image point_cloud_image = NULL;
            point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                                                   color_image_width_pixels,
                                                   color_image_height_pixels,
                                                   color_image_width_pixels * 3 * (int)sizeof(int16_t));
            transformation.depth_image_to_color_camera(depth_image, &transformed_depth_image);
            transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

            int width = color_image.get_width_pixels();
            int height = color_image.get_height_pixels();
            captured_cloud->reserve(width *height);

            int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
            uint8_t* color_image_data = color_image.get_buffer();
            Eigen::Matrix3f m;
            m = Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitZ());
            for (int i = 0; i < width * height; ++i){
                PointXYZRGBN point;
                point.x = point_cloud_image_data[3 * i + 0];
                point.y = point_cloud_image_data[3 * i + 1];
                point.z = point_cloud_image_data[3 * i + 2];
                if (point.z == 0){
                    continue;
                }
                point.getVector3fMap() = m * point.getVector3fMap();
                point.b = color_image_data[4 * i + 0];
                point.g = color_image_data[4 * i + 1];
                point.r = color_image_data[4 * i + 2];
                uint8_t alpha = color_image_data[4 * i + 3];
                if (point.b == 0 && point.g == 0 && point.r == 0 && alpha == 0){
                    continue;
                }
                captured_cloud->points.push_back(point);
            }
            cloud_view->addCloud(captured_cloud,"k4a");
            if(ui->check_withoutcolor->isChecked())
                cloud_view->setCloudColor(captured_cloud,"k4a","z");
        } else if(output_type==1) {
            cv_rgbImage->copy(cv::Mat(color_image.get_height_pixels(), color_image.get_width_pixels(), CV_8UC4, color_image.get_buffer()));
            image_view->addImage(cv_rgbImage,false);
        } else if(output_type==2) {
            cv_depth->copy(cv::Mat(depth_image.get_height_pixels(), depth_image.get_width_pixels(), CV_8UC4,depth_image.get_buffer()));
            image_view->addImage(cv_depth,false);
        }else if(output_type==3) {
            cv_irImage->copy(cv::Mat(infrared_image.get_height_pixels(), infrared_image.get_width_pixels(), CV_8UC4,infrared_image.get_buffer()));
            image_view->addImage(cv_depth,false);
        }
    } catch (k4a::error error){
        console->error(error.what());
        return;
    }
}

void DeviceKinectDK::closeEvent(QCloseEvent *event)
{
    capture_timer->stop();
    image_view->removeImage();
    cloud_view->removeCloud("k4a");
    if(device)
        device.close();
    transformation.destroy();
    return QDockWidget::closeEvent(event);
}
