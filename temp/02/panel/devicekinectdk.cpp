#include "devicekinectdk.h"
#include "ui_devicekinectdk.h"

DeviceKinectDK::DeviceKinectDK(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DeviceKinectDK),
    capturedCloud(new CloudXYZRGBN),
    outputType(0),
    isConnected(false),
    isCapturing(false),
    config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL)
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

void DeviceKinectDK::init()
{
    connect(ui->cbox_depth,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        config.depth_mode=k4a_depth_mode_t(index+1);
        if(index==3){
            if(ui->rbtn_30fps->isChecked()){
                ui->rbtn_15fps->setChecked(true);
                config.camera_fps=K4A_FRAMES_PER_SECOND_15;
            }
            ui->rbtn_30fps->setEnabled(false);
        }
        else {
            ui->rbtn_30fps->setEnabled(true);
        }
    });
    connect(ui->cbox_colorformat,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        config.color_format=k4a_image_format_t(index);
        if((index==1)|(index==2)){
            ui->cbox_resolution->setCurrentIndex(0);
            ui->cbox_resolution->setEnabled(false);
            config.color_resolution=k4a_color_resolution_t(1);
        }
        else{
            ui->cbox_resolution->setEnabled(true);
        }
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
        else{
            ui->rbtn_30fps->setEnabled(true);
        }
    });
    connect(ui->cbox_output,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        outputType=index;
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
        if(isCapturing){
            captureTimer->stop();
            captureTimer->start(value);
        }
    });
    captureTimer=new QTimer();
    connect(captureTimer,&QTimer::timeout,[=]{
        try{
            time.tic();
            if (device.get_capture(&capture, std::chrono::milliseconds(500))){
                depthImage = capture.get_depth_image();
                colorImage = capture.get_color_image();
                infraredImage=capture.get_ir_image();
                if(depthImage==nullptr|colorImage==nullptr|infraredImage==nullptr){
                    console->error(tr("Failed to capture,retry"));
                    return;
                }
            }
            if(outputType==0) {
                this->captureCloud();
                cloudView->showInfoText(std::to_string(time.toc())+" ms",30,"info");
            }
            else
                this->captureImage(outputType);
        }
        catch (k4a::error error){
            console->error(error.what());
            captureTimer->stop();
            return;
        }
    });

    ui->cbox_depth->setCurrentIndex(0);
    config.depth_mode=k4a_depth_mode_t(1);
    ui->cbox_resolution->setCurrentIndex(1);
    ui->cbox_colorformat->setCurrentIndex(3);
}

void DeviceKinectDK::connectDevice()
{
    if(!isConnected){
        int device_count = k4a::device::get_installed_count();
        if(device_count<=0){
            console->warning(tr("Found 0 connected device."));
            return ;
        }
        console->info(tr("Found %1 connected device.").arg(device_count));
        for(int i=0;i<device_count;i++){
            try{
                device=k4a::device::open(i);
                device.start_cameras(&config);
                calibration = device.get_calibration(config.depth_mode, config.color_resolution);
                transformation = k4a::transformation(calibration);
                console->info(tr("Open the device sucessfully!"));
                ui->btn_connect->setText(tr("Discon"));
                ui->btn_connect->setIcon(QIcon(":/icon/resource/icon/disconnect.svg"));
                isConnected=true;

            } catch (k4a::error error){
                console->error(error.what());
                isConnected=false;
                return;
            }
        }
    } else {
        if(isCapturing){
            isCapturing=false;
            cloudView->removeShape("info");
            captureTimer->stop();
            ui->btn_capture->setText(tr("Capture"));
            ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        }
        if(cloudView->contains("k4a"))
            cloudView->removeCloud("k4a");
        cloudView->removeShape("info");
        try{
            if(device)
                device.close();
            transformation.destroy();
            console->info(tr("Close the device sucessfully!"));
            ui->btn_connect->setText(tr("Connect"));
            ui->btn_connect->setIcon(QIcon(":/icon/resource/icon/connect.svg"));
            isConnected=false;
        }
        catch (k4a::error error){
            console->error(error.what());
            isConnected=false;
            return;
        }
    }
}

void DeviceKinectDK::captureDevice()
{
    if(isConnected){
        if(!ui->checkbox_autofresh->isChecked()){
            console->info(tr("Capturing..."));
            try{
                if (device.get_capture(&capture, std::chrono::milliseconds(500))){
                    depthImage = capture.get_depth_image();
                    colorImage = capture.get_color_image();
                    infraredImage=capture.get_ir_image();
                    if(depthImage==nullptr|colorImage==nullptr|infraredImage==nullptr){
                        console->error(tr("Failed to capture,retry"));
                        return;
                    }
                }
                if(outputType==0)
                    this->captureCloud();
                else
                    this->captureImage(outputType);
            }
            catch (k4a::error error){
                console->error(error.what());
                return;
            }
        } else {
            if(!isCapturing){
                console->info(tr("Capturing..."));
                captureTimer->start(ui->slider_refreshtime->value());
                isCapturing=true;
                ui->btn_capture->setText(tr("Stop"));
                ui->btn_capture->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
            }
            else{
                isCapturing=false;
                captureTimer->stop();
                cloudView->removeShape("info");
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
    QFileInfo fileinfo("../KinectDK/kinectdk-"+QDateTime::currentDateTime().toString("hh-mm-ss"));
    if(outputType==0){
        if(capturedCloud->empty()) {
            console->warning(tr("Please capture a pointcloud."));
            return;
        }
        Cloud cloud(capturedCloud,fileinfo);
        cloudView->removeCloud("k4a");
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
        if(cv_depth.empty()){
            console->warning(tr("Please capture a depth data."));
            return;
        }
        Image image(cv_depth,fileinfo);
        imageTree->insertImage(-1,image,true);
    }else if(outputType==3) {
        if(cv_irImage.empty()){
            console->warning(tr("Please capture a infrared data."));
            return;
        }
        Image image(cv_irImage,fileinfo);
        imageTree->insertImage(-1,image,true);
    }
}

void DeviceKinectDK::captureCloud()
{
    capturedCloud.reset(new CloudXYZRGBN);
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

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

    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    capturedCloud->points.reserve(width *height);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();

#ifdef VTK_VISUALIZATION
    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitZ());
#endif

    for (int i = 0; i < width * height; ++i){
        PointXYZRGBN point;
        point.x = point_cloud_image_data[3 * i + 0];
        point.y = point_cloud_image_data[3 * i + 1];
        point.z = point_cloud_image_data[3 * i + 2];
        if (point.z == 0){
            continue;
        }

#ifdef VTK_VISUALIZATION
        point.getVector3fMap() = m * point.getVector3fMap();
#endif
        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];
        if (point.b == 0 && point.g == 0 && point.r == 0 && alpha == 0){
            continue;
        }
        capturedCloud->points.push_back(point);
    }
    cloudView->updateCloud(capturedCloud,"k4a");
    if(ui->checkbox_withoutcolor->isChecked())
        cloudView->setCloudColor(capturedCloud,"k4a","z");
}

void DeviceKinectDK::captureImage(int index)
{
    switch (index){
    case 1:
        colorTextureBuffer= colorImage.get_buffer();
        cv_rgbImage = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
        graphicsView->addImage(cv_rgbImage);
        break;
    case 2:
        this->ColorizeDepthImage(depthImage,this->ColorizeBlueToRed, this->GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
        cv_depth = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
        graphicsView->addImage(cv_depth);
        break;
    case 3:
        this->ColorizeDepthImage(infraredImage, this->ColorizeGreyscale, this->GetIrLevels(config.depth_mode), &irTextureBuffer);
        cv_irImage = cv::Mat(infraredImage.get_height_pixels(), infraredImage.get_width_pixels(), CV_8UC4, irTextureBuffer.data());
        graphicsView->addImage(cv_irImage);
        break;
    }
}

void DeviceKinectDK::closeEvent(QCloseEvent *event)
{
    captureTimer->stop();
    graphicsView->removeImage();
    cloudView->removeCloud("k4a");
    cloudView->removeShape("info");
    if(device)
        device.close();
    transformation.destroy();
    cv_rgbImage.release();
    cv_depth.release();
    cv_irImage.release();
    return QWidget::closeEvent(event);
}

std::pair<uint16_t, uint16_t> DeviceKinectDK::GetDepthModeRange(const k4a_depth_mode_t depthMode)
{
    switch (depthMode)
    {
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
        return { (uint16_t)500, (uint16_t)5800 };
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
        return { (uint16_t)500, (uint16_t)4000 };
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
        return { (uint16_t)250, (uint16_t)3000 };
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
        return { (uint16_t)250, (uint16_t)2500 };

    case K4A_DEPTH_MODE_PASSIVE_IR:
    default:
        throw std::logic_error("Invalid depth mode!");
    }
}

std::pair<int, int> DeviceKinectDK::GetDepthDimensions(const k4a_depth_mode_t depthMode)
{
    switch (depthMode)
    {
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
        return { 320, 288 };
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
        return { 640, 576 };
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
        return { 512, 512 };
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
        return { 1024, 1024 };
    case K4A_DEPTH_MODE_PASSIVE_IR:
        return { 1024, 1024 };

    default:
        throw std::logic_error("Invalid depth dimensions value!");
    }
}

std::pair<int, int> DeviceKinectDK::GetColorDimensions(const k4a_color_resolution_t resolution)
{
    switch (resolution)
    {
    case K4A_COLOR_RESOLUTION_720P:
        return { 1280, 720 };
    case K4A_COLOR_RESOLUTION_2160P:
        return { 3840, 2160 };
    case K4A_COLOR_RESOLUTION_1440P:
        return { 2560, 1440 };
    case K4A_COLOR_RESOLUTION_1080P:
        return { 1920, 1080 };
    case K4A_COLOR_RESOLUTION_3072P:
        return { 4096, 3072 };
    case K4A_COLOR_RESOLUTION_1536P:
        return { 2048, 1536 };

    default:
        throw std::logic_error("Invalid color dimensions value!");
    }
}

std::pair<uint16_t, uint16_t> DeviceKinectDK::GetIrLevels(const k4a_depth_mode_t depthMode)
{
    switch (depthMode)
    {
    case K4A_DEPTH_MODE_PASSIVE_IR:
        return { (uint16_t)0, (uint16_t)100 };

    case K4A_DEPTH_MODE_OFF:
        throw std::logic_error("Invalid depth mode!");

    default:
        return { (uint16_t)0, (uint16_t)1000 };
    }
}

void DeviceKinectDK::ColorizeDepthImage(const k4a::image &depthImage, DepthPixelVisualizationFunction visualizationFn, std::pair<uint16_t, uint16_t> expectedValueRange, std::vector<pixel> *buffer)
{
    const k4a_image_format_t imageFormat = depthImage.get_format();
    if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)
    {
        throw std::logic_error("Attempted to colorize a non-depth image!");
    }

    const int width = depthImage.get_width_pixels();
    const int height = depthImage.get_height_pixels();

    buffer->resize(static_cast<size_t>(width * height));

    const uint16_t* depthData = reinterpret_cast<const uint16_t*>(depthImage.get_buffer());
    for (int h = 0; h < height; ++h)
    {
        for (int w = 0; w < width; ++w)
        {
            const size_t currentPixel = static_cast<size_t>(h * width + w);
            (*buffer)[currentPixel] = visualizationFn(depthData[currentPixel], expectedValueRange.first, expectedValueRange.second);
        }
    }
}

pixel DeviceKinectDK::ColorizeBlueToRed(const uint16_t &depthPixel, const uint16_t &min, const uint16_t &max)
{
    constexpr uint8_t PixelMax = std::numeric_limits<uint8_t>::max();
    pixel result = { uint8_t(0), uint8_t(0), uint8_t(0), PixelMax };
    if (depthPixel == 0)
    {
        return result;
    }

    uint16_t clampedValue = depthPixel;
    clampedValue = std::min(clampedValue, max);
    clampedValue = std::max(clampedValue, min);

    float hue = (clampedValue - min) / static_cast<float>(max - min);
    constexpr float range = 2.f / 3.f;
    hue *= range;
    hue = range - hue;
    float fRed = 0.f;
    float fGreen = 0.f;
    float fBlue = 0.f;
    //ConvertHSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
    hue = fmodf(hue, 1.0f) / (60.0f / 360.0f);
    int   i = (int)hue;
    float f = hue - (float)i;
    float p = 1.f* (1.0f - 1.f);
    float q = 1.f* (1.0f - 1.f * f);
    float t = 1.f* (1.0f -1.f* (1.0f - f));

    switch (i)
    {
    case 0: fRed = 1.f; fGreen = t; fBlue = p; break;
    case 1: fRed = q; fGreen = 1.f; fBlue = p; break;
    case 2: fRed = p; fGreen = 1.f; fBlue = t; break;
    case 3: fRed = p; fGreen = q; fBlue = 1.f; break;
    case 4: fRed = t; fGreen = p; fBlue = 1.f; break;
    case 5: default: fRed = 1.f; fGreen = p; fBlue = q; break;
    }
    result.Red = static_cast<uint8_t>(fRed * PixelMax);
    result.Green = static_cast<uint8_t>(fGreen * PixelMax);
    result.Blue = static_cast<uint8_t>(fBlue * PixelMax);
    return result;
}

pixel DeviceKinectDK::ColorizeGreyscale(const uint16_t &value, const uint16_t &min, const uint16_t &max)
{
    uint16_t pixelValue = std::min(value, max);
    constexpr uint8_t PixelMax = std::numeric_limits<uint8_t>::max();
    const auto normalizedValue = static_cast<uint8_t>((pixelValue - min) * (double(PixelMax) / (max - min)));
    return pixel{ normalizedValue, normalizedValue, normalizedValue, PixelMax };
}

