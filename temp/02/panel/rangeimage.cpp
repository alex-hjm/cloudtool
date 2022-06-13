#include "rangeimage.h"
#include "ui_rangeimage.h"

RangeImage::RangeImage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RangeImage),
    angularResolution(0.5f),
    noiseLevel(0.5f),
    minRange(0.0f),
    borderSize(1),
    coordinateFrame(pcl::RangeImage::CAMERA_FRAME)
{
    ui->setupUi(this);
    rangeImagePtr.reset(new pcl::RangeImage);
}

RangeImage::~RangeImage()
{
    delete ui;
}

void RangeImage::init()
{
    connect(cloudView,&CloudView::viewerPose,this,&RangeImage::updateRangeImage);
    cloudTree->setExtendedSelection(false);
}

void RangeImage::updateRangeImage(Eigen::Affine3f &viewer_pose)
{
    selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    noiseLevel=ui->dspin_noise_level->value();
    minRange=ui->dspin_min_range->value();
    borderSize=ui->spin_border_size->value();
    rangeImagePtr->createFromPointCloud(*selectedCloud.cloud, pcl::deg2rad(angularResolution), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
            viewer_pose,  coordinateFrame, noiseLevel, minRange, borderSize);

    floatImage = rangeImagePtr->getRangesArray ();
    rgbImage = pcl::visualization::FloatImageUtils::getVisualImage (floatImage, rangeImagePtr->width, rangeImagePtr->height);
    data = const_cast<void*> (reinterpret_cast<const void*> (rgbImage));

    image = vtkSmartPointer<vtkImageData>::New ();
    image->SetExtent(0, rangeImagePtr->width- 1, 0,  rangeImagePtr->height - 1, 0,0);
    image->AllocateScalars (VTK_UNSIGNED_CHAR, 3);
    image->GetPointData ()->GetScalars ()->SetVoidArray (data, 3 * rangeImagePtr->width *  rangeImagePtr->height, 1);

    actor =	vtkSmartPointer<vtkImageActor>::New();
    actor->SetInputData(image);

    render = vtkSmartPointer<vtkRenderer>::New();
    render->AddActor(actor);
    render->ResetCamera();
    render->SetBackground(1,1,1);

    ui->qvtkWidget->GetRenderWindow()->RemoveRenderer(render);
    ui->qvtkWidget->GetRenderWindow()->AddRenderer(render);
    ui->qvtkWidget->GetRenderWindow()->Render();
}

void RangeImage::closeEvent(QCloseEvent *event)
{
    cloudTree->setExtendedSelection(true);
    disconnect(cloudView,&CloudView::viewerPose,this,&RangeImage::updateRangeImage);
}
