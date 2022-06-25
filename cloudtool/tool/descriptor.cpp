/**
 * @file descriptor.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "descriptor.h"
#include "ui_descriptor.h"

#define DESCRIPTOR_TYPE_PFHEstimation               (0)
#define DESCRIPTOR_TYPE_FPFHEstimation              (1)
#define DESCRIPTOR_TYPE_VFHEstimation               (2)
#define DESCRIPTOR_TYPE_ESFEstimation               (3)
#define DESCRIPTOR_TYPE_GASDEstimation              (4)
#define DESCRIPTOR_TYPE_GASDColorEstimation         (5)
#define DESCRIPTOR_TYPE_RSDEstimation               (6)
#define DESCRIPTOR_TYPE_GRSDEstimation              (7)
#define DESCRIPTOR_TYPE_CRHEstimation               (8)
#define DESCRIPTOR_TYPE_CVFHEstimation              (9)
#define DESCRIPTOR_TYPE_ShapeContext3DEstimation    (10)
#define DESCRIPTOR_TYPE_SHOTEstimation              (11)
#define DESCRIPTOR_TYPE_SHOTColorEstimation         (12)
#define DESCRIPTOR_TYPE_UniqueShapeContext          (13)
#define DESCRIPTOR_TYPE_BOARDLocalReferenceFrameEstimation  (14)
#define DESCRIPTOR_TYPE_FLARELocalReferenceFrameEstimation  (15)
#define DESCRIPTOR_TYPE_SHOTLocalReferenceFrameEstimation   (16)

Descriptor::Descriptor(QWidget* parent)
    :CustomDock(parent), ui(new Ui::Descriptor),
    m_thread(this),
    m_plotter(new pcl::visualization::PCLPlotter)
{
    ui->setupUi(this);

    qRegisterMetaType<ct::FeatureType::Ptr>("FeatureType::Ptr &");
    qRegisterMetaType<ct::FeatureType::Ptr>("FeatureType::Ptr");
    qRegisterMetaType<ct::ReferenceFrame::Ptr>("ReferenceFrame::Ptr &");
    qRegisterMetaType<ct::ReferenceFrame::Ptr>("ReferenceFrame::Ptr");

    connect(ui->btn_preview, &QPushButton::clicked, this, &Descriptor::preview);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Descriptor::reset);

    m_features = new ct::Features;
    m_features->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_features, &QObject::deleteLater);
    connect(this, &Descriptor::PFHEstimation, m_features, &ct::Features::PFHEstimation);
    connect(this, &Descriptor::FPFHEstimation, m_features, &ct::Features::FPFHEstimation);
    connect(this, &Descriptor::VFHEstimation, m_features, &ct::Features::VFHEstimation);
    connect(this, &Descriptor::ESFEstimation, m_features, &ct::Features::ESFEstimation);
    connect(this, &Descriptor::GASDEstimation, m_features, &ct::Features::GASDEstimation);
    connect(this, &Descriptor::GASDColorEstimation, m_features, &ct::Features::GASDColorEstimation);
    connect(this, &Descriptor::RSDEstimation, m_features, &ct::Features::RSDEstimation);
    connect(this, &Descriptor::GRSDEstimation, m_features, &ct::Features::GRSDEstimation);
    connect(this, &Descriptor::CRHEstimation, m_features, &ct::Features::CRHEstimation);
    connect(this, &Descriptor::CVFHEstimation, m_features, &ct::Features::CVFHEstimation);
    connect(this, &Descriptor::ShapeContext3DEstimation, m_features, &ct::Features::ShapeContext3DEstimation);
    connect(this, &Descriptor::SHOTEstimation, m_features, &ct::Features::SHOTEstimation);
    connect(this, &Descriptor::SHOTColorEstimation, m_features, &ct::Features::SHOTColorEstimation);
    connect(this, &Descriptor::UniqueShapeContext, m_features, &ct::Features::UniqueShapeContext);
    connect(this, &Descriptor::BOARDLocalReferenceFrameEstimation, m_features, &ct::Features::BOARDLocalReferenceFrameEstimation);
    connect(this, &Descriptor::FLARELocalReferenceFrameEstimation, m_features, &ct::Features::FLARELocalReferenceFrameEstimation);
    connect(this, &Descriptor::SHOTLocalReferenceFrameEstimation, m_features, &ct::Features::SHOTLocalReferenceFrameEstimation);
    connect(m_features, &ct::Features::featureResult, this, &Descriptor::featureResult);
    connect(m_features, &ct::Features::lrfResult, this, &Descriptor::lrfResult);
    m_thread.start();

    connect(ui->cbox_feature, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int index)
            {
                if (index == 0)
                {
                    ui->cbox_type->show();
                    ui->cbox_lrf->hide();
                    ui->stackedWidget->setCurrentIndex(ui->cbox_type->currentIndex());
                }
                else
                {
                    ui->cbox_type->hide();
                    ui->cbox_lrf->show();
                    ui->stackedWidget->setCurrentIndex(ui->cbox_lrf->currentIndex() + 14);

                }
            });
    connect(ui->cbox_lrf, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int index)
            {ui->stackedWidget->setCurrentIndex(14 + index);});

    ui->cbox_type->setCurrentIndex(0);
    ui->cbox_feature->setCurrentIndex(0);
    ui->cbox_lrf->hide();
}

Descriptor::~Descriptor()
{
    m_thread.quit();
    m_thread.wait();
    delete ui;
}

void Descriptor::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (!m_plotter->wasStopped()) m_plotter->close();
    m_plotter.reset(new pcl::visualization::PCLPlotter);
    for (auto& cloud : selected_clouds)
    {
        if (ui->spin_k->value() == 0 && ui->dspin_r->value() == 0)
        {
            printW("Parameters set error!");
            return;
        }
        m_features->setInputCloud(cloud);
        m_features->setKSearch(ui->spin_k->value());
        m_features->setRadiusSearch(ui->dspin_r->value());
        switch (ui->cbox_type->currentIndex())
        {
        case DESCRIPTOR_TYPE_PFHEstimation:
            m_cloudview->showInfo("PFHEstimation", 1);
            emit PFHEstimation();
            break;
        case DESCRIPTOR_TYPE_FPFHEstimation:
            m_cloudview->showInfo("FPFHEstimation", 1);
            emit FPFHEstimation();
            break;
        case DESCRIPTOR_TYPE_VFHEstimation:
            m_cloudview->showInfo("VFHEstimation", 1);
            emit VFHEstimation(Eigen::Vector3f(ui->dspin_vpx1->value(), ui->dspin_vpy1->value(), ui->dspin_vpz1->value()));
            break;
        case DESCRIPTOR_TYPE_ESFEstimation:
            m_cloudview->showInfo("ESFEstimation", 1);
            emit ESFEstimation();
            break;
        case DESCRIPTOR_TYPE_GASDEstimation:
            m_cloudview->showInfo("GASDEstimation", 1);
            emit GASDEstimation(Eigen::Vector3f(ui->dspin_vx1->value(), ui->dspin_vy1->value(), ui->dspin_vz1->value()),
                                ui->spin_shgs1->value(), ui->spin_shs1->value(), ui->cbox_interp1->currentIndex());
            break;
        case DESCRIPTOR_TYPE_GASDColorEstimation:
            m_cloudview->showInfo("GASDColorEstimation", 1);
            emit GASDColorEstimation(Eigen::Vector3f(ui->dspin_vx2->value(), ui->dspin_vy2->value(), ui->dspin_vz2->value()),
                                     ui->spin_shgs2->value(), ui->spin_shs2->value(), ui->cbox_interp2->currentIndex(),
                                     ui->spin_chgs->value(), ui->spin_chs->value(), ui->cbox_cinterp->currentIndex());
            break;
        case DESCRIPTOR_TYPE_RSDEstimation:
            m_cloudview->showInfo("RSDEstimation", 1);
            emit RSDEstimation(ui->spin_nr_subdiv->value(), ui->dspin_plane_radius->value());
            break;
        case DESCRIPTOR_TYPE_GRSDEstimation:
            m_cloudview->showInfo("GRSDEstimation", 1);
            emit GRSDEstimation();
            break;
        case DESCRIPTOR_TYPE_CRHEstimation:
            m_cloudview->showInfo("CRHEstimation", 1);
            emit CRHEstimation(Eigen::Vector3f(ui->dspin_vpx2->value(), ui->dspin_vpy2->value(), ui->dspin_vpz2->value()));
            break;
        case DESCRIPTOR_TYPE_CVFHEstimation:
            m_cloudview->showInfo("CVFHEstimation", 1);
            emit CVFHEstimation(Eigen::Vector3f(ui->dspin_vpx2->value(), ui->dspin_vpy2->value(), ui->dspin_vpz2->value()),
                                ui->dspin_rn->value(), ui->dspin_d1->value(), ui->dspin_d2->value(), ui->dspin_d3->value(),
                                ui->spin_min->value(), ui->check_normalize->isChecked());
            break;
        case DESCRIPTOR_TYPE_ShapeContext3DEstimation:
            m_cloudview->showInfo("ShapeContext3DEstimation", 1);
            emit ShapeContext3DEstimation(ui->dspin_min_r->value(), ui->dspin_r2->value());
            break;
        case DESCRIPTOR_TYPE_SHOTEstimation:
            if (m_lrf_map.find(cloud->id()) == m_lrf_map.end())
            {
                printW("Please Estimation LocalReferenceFrame first!");
                return;
            }
            m_cloudview->showInfo("SHOTEstimation", 1);
            emit SHOTEstimation(m_lrf_map.find(cloud->id())->second, ui->dspin_lrf1->value());
            break;
        case DESCRIPTOR_TYPE_SHOTColorEstimation:
            if (m_lrf_map.find(cloud->id()) == m_lrf_map.end())
            {
                printW("Please Estimation LocalReferenceFrame first!");
                return;
            }
            m_cloudview->showInfo("SHOTColorEstimation", 1);
            emit SHOTColorEstimation(m_lrf_map.find(cloud->id())->second, ui->dspin_lrf2->value());
            break;
        case DESCRIPTOR_TYPE_UniqueShapeContext:
            if (m_lrf_map.find(cloud->id()) == m_lrf_map.end())
            {
                printW("Please Estimation LocalReferenceFrame first!");
                return;
            }
            m_cloudview->showInfo("UniqueShapeContext", 1);
            emit UniqueShapeContext(m_lrf_map.find(cloud->id())->second, ui->dspin_min_r2->value(), ui->dspin_p_r->value(),
                                    ui->dspin_l_r->value());
            break;
        case DESCRIPTOR_TYPE_BOARDLocalReferenceFrameEstimation:
            m_cloudview->showInfo("BOARDLocalReferenceFrameEstimation", 1);
            emit BOARDLocalReferenceFrameEstimation(ui->dspin_t_r1->value(), ui->check_find_holes->isChecked(), ui->dspin_m_t1->value(),
                                                    ui->spin_size->value(), ui->dspin_h_t1->value(), ui->dspin_s_t1->value());
            break;
        case DESCRIPTOR_TYPE_FLARELocalReferenceFrameEstimation:
            m_cloudview->showInfo("FLARELocalReferenceFrameEstimation", 1);
            emit FLARELocalReferenceFrameEstimation(ui->dspin_t_r2->value(), ui->dspin_m_t2->value(), ui->spin_m_n2->value(), ui->spin_m_t2->value());
            break;
        case DESCRIPTOR_TYPE_SHOTLocalReferenceFrameEstimation:
            m_cloudview->showInfo("SHOTLocalReferenceFrameEstimation", 1);
            emit SHOTLocalReferenceFrameEstimation();
            break;
        }
        m_cloudtree->showProgressBar();
    }
}

void Descriptor::reset()
{
    m_plotter.reset(new pcl::visualization::PCLPlotter);
    m_plotter->close();
    m_plotter = nullptr;
    m_descriptor_map.clear();
    m_lrf_map.clear();
    m_cloudview->clearInfo();
}

void Descriptor::featureResult(const QString& id, const ct::FeatureType::Ptr& feature, float time)
{
    m_descriptor_map[id] = feature;
    m_cloudtree->closeProgressBar();
    switch (ui->cbox_type->currentIndex())
    {
    case DESCRIPTOR_TYPE_PFHEstimation:
        printI(QString("Estimate cloud[id:%1] PFHFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("PFHEstimation");
        m_plotter->addFeatureHistogram(*feature->pfh, 1000);
        break;
    case DESCRIPTOR_TYPE_FPFHEstimation:
        printI(QString("Estimate cloud[id:%1] FPFHFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("FPFHEstimation");
        m_plotter->addFeatureHistogram(*feature->fpfh, 1000);
        break;
    case DESCRIPTOR_TYPE_VFHEstimation:
        printI(QString("Estimate cloud[id:%1] VFHFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("VFHEstimation");
        m_plotter->addFeatureHistogram(*feature->vfh, 1000);
        break;
    case DESCRIPTOR_TYPE_ESFEstimation:
        printI(QString("Estimate cloud[id:%1] ESFHFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("ESFEstimation");
        m_plotter->addFeatureHistogram(*feature->esf, 1000);
        break;
    case DESCRIPTOR_TYPE_GASDEstimation:
        printI(QString("Estimate cloud[id:%1] GASDFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("GASDEstimation");
        m_plotter->addFeatureHistogram(*feature->gasd, 1000);
        break;
    case DESCRIPTOR_TYPE_GASDColorEstimation:
        printI(QString("Estimate cloud[id:%1] GASDCFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("GASDColorEstimation");
        m_plotter->addFeatureHistogram(*feature->gasdc, 1000);
        break;
    case DESCRIPTOR_TYPE_RSDEstimation:
        printI(QString("Estimate cloud[id:%1] RSDFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("RSDEstimation");
        //m_plotter->addFeatureHistogram(*feature->rsd, 1000);
        break;
    case DESCRIPTOR_TYPE_GRSDEstimation:
        printI(QString("Estimate cloud[id:%1] GRSDFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("GRSDEstimation");
        m_plotter->addFeatureHistogram(*feature->grsd, 1000);
        break;
    case DESCRIPTOR_TYPE_CRHEstimation:
        printI(QString("Estimate cloud[id:%1] CRHFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("CRHEstimation");
        m_plotter->addFeatureHistogram(*feature->crh, 1000);
        break;
    case DESCRIPTOR_TYPE_CVFHEstimation:
        printI(QString("Estimate cloud[id:%1] CVFHFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("CVFHEstimation");
        m_plotter->addFeatureHistogram(*feature->vfh, 1000);
        break;
    case DESCRIPTOR_TYPE_ShapeContext3DEstimation:
        printI(QString("Estimate cloud[id:%1] SC3DFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("ShapeContext3DEstimation");
        //m_plotter->addFeatureHistogram(*feature->sc3d, 1000);
        break;
    case DESCRIPTOR_TYPE_SHOTEstimation:
        printI(QString("Estimate cloud[id:%1] SHOTFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("SHOTEstimation");
        //m_plotter->addFeatureHistogram(*feature->shot, 1000);
        break;
    case DESCRIPTOR_TYPE_SHOTColorEstimation:
        printI(QString("Estimate cloud[id:%1] SHOTCFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("SHOTColorEstimation");
        //m_plotter->addFeatureHistogram(*feature->shotc, 1000);
        break;
    case DESCRIPTOR_TYPE_UniqueShapeContext:
        printI(QString("Estimate cloud[id:%1] USCFeature done, take time %2 ms.").arg(id).arg(time));
        m_plotter->setTitle("UniqueShapeContext");
        //m_plotter->addFeatureHistogram(*feature->usc, 1000);
        break;
    }
    QPoint pos = m_cloudview->mapToGlobal(QPoint((m_cloudview->width() - 640) / 2, (m_cloudview->height() - 200) / 2));
    m_plotter->setWindowPosition(pos.x(), pos.y());
    m_plotter->plot();
}

void Descriptor::lrfResult(const QString& id, const ct::ReferenceFrame::Ptr& cloud, float time)
{
    switch (ui->cbox_type->currentIndex())
    {
    case DESCRIPTOR_TYPE_BOARDLocalReferenceFrameEstimation:
        printI(QString("Estimate cloud[id:%1] BOARD LocalReferenceFrame done, take time %2 ms.").arg(id).arg(time));
        break;
    case DESCRIPTOR_TYPE_FLARELocalReferenceFrameEstimation:
        printI(QString("Estimate cloud[id:%1] FLARE LocalReferenceFrame done, take time %2 ms.").arg(id).arg(time));
        break;
    case DESCRIPTOR_TYPE_SHOTLocalReferenceFrameEstimation:
        printI(QString("Estimate cloud[id:%1] SHOT LocalReferenceFrame done, take time %2 ms.").arg(id).arg(time));
        break;
    }
    m_lrf_map[id] = cloud;
    m_cloudtree->closeProgressBar();
}