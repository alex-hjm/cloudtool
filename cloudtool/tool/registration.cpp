#include "registration.h"
#include "ui_registration.h"

#define REG_TYPE_CorrespondenceEstimation       (0)
#define REG_TYPE_CorrespondenceRejector         (1)
#define REG_TYPE_TransformationEstimation       (2)
#define REG_TYPE_Registration                   (3)

#define REG_TYPE_CE_Base                        (0)
#define REG_TYPE_CE_BackProjection              (1)
#define REG_TYPE_CE_NormalShooting              (2)

#define REG_TYPE_CR_Distance                    (0)
#define REG_TYPE_CR_MedianDistance              (1)
#define REG_TYPE_CR_OneToOne                    (2)
#define REG_TYPE_CR_OrganizedBoundary           (3)
#define REG_TYPE_CR_Poly                        (4)
#define REG_TYPE_CR_SampleConsensus             (5)
#define REG_TYPE_CR_SurfaceNormal               (6)
#define REG_TYPE_CR_Trimmed                     (7)
#define REG_TYPE_CR_VarTrimmed                  (8)

#define REG_TYPE_TE_2D                          (0)
#define REG_TYPE_TE_3Point                      (1)
#define REG_TYPE_TE_DualQuaternion              (2)
#define REG_TYPE_TE_LM                          (3)
#define REG_TYPE_TE_PointToPlane                (4)
#define REG_TYPE_TE_PointToPlaneLLS             (5)
#define REG_TYPE_TE_SVD                         (6)
#define REG_TYPE_TE_SymmetricPointToPlaneLLS    (7)

#define REG_TYPE_PointCloud                     (0)
#define REG_TYPE_PFHFeature                     (1)
#define REG_TYPE_FPFHFeature                    (2)

#define REG_IterativeClosestPoint               (0)
#define REG_IterativeClosestPointWithNormals    (1)
#define REG_IterativeClosestPointNonLinear      (2)
#define REG_GeneralizedIterativeClosestPoint    (3)
#define REG_SampleConsensusInitialAlignment     (4)
#define REG_SampleConsensusPrerejective         (5)
#define REG_NormalDistributionsTransform        (6)
#define REG_FPCSInitialAlignment                (7)
#define REG_KFPCSInitialAlignment               (8)


#define REG_CORRE_PRE_FLAG                      "-corre"
#define REG_TRANS_PRE_FLAG                      "-trans"
#define REG_ALIGN_PRE_FLAG                      "-align"
#define REG_ALIGN_ADD_FLAG                      "align-"

Registration::Registration(QWidget* parent)
    :CustomDock(parent), ui(new Ui::Registration),
    m_thread(this),
    m_target_cloud(nullptr),
    m_source_cloud(nullptr),
    m_corr(nullptr),
    m_ce(nullptr)
{
    ui->setupUi(this);

    qRegisterMetaType<ct::CorrespondencesPtr>("CorrespondencesPtr &");
    qRegisterMetaType<ct::CorrespondencesPtr>("CorrespondencesPtr");
    qRegisterMetaType<ct::CorreEst::Ptr>("CorreEst::Ptr &");
    qRegisterMetaType<ct::CorreEst::Ptr>("CorreEst::Ptr");
    qRegisterMetaType<ct::CorreRej::Ptr>("CorreRej::Ptr &");
    qRegisterMetaType<ct::CorreRej::Ptr>("CorreRej::Ptr");
    qRegisterMetaType<ct::TransEst::Ptr>("TransEst::Ptr &");
    qRegisterMetaType<ct::TransEst::Ptr>("TransEst::Ptr");
    qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f &");
    qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");

    connect(ui->btn_setTarget, &QPushButton::clicked, this, &Registration::setTarget);
    connect(ui->btn_setSource, &QPushButton::clicked, this, &Registration::setSource);
    connect(ui->btn_setCE, &QPushButton::clicked, this, &Registration::setCorrespondenceEstimation);
    connect(ui->btn_addCR, &QPushButton::clicked, this, &Registration::addCorrespondenceRejector);
    connect(ui->btn_removeCR, &QPushButton::clicked, this, &Registration::removeCorrespondenceRejector);
    connect(ui->btn_setTE, &QPushButton::clicked, this, &Registration::setTransformationEstimation);
    connect(ui->btn_preview, &QPushButton::clicked, this, &Registration::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Registration::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Registration::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Registration::reset);

    m_reg = new ct::Registration;
    m_reg->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_reg, &QObject::deleteLater);
    connect(this, &Registration::CorrespondenceEstimationBackProjection, m_reg, &ct::Registration::CorrespondenceEstimationBackProjection);
    connect(this, &Registration::CorrespondenceEstimationNormalShooting, m_reg, &ct::Registration::CorrespondenceEstimationNormalShooting);
    connect(this, &Registration::CorrespondenceRejectorDistance, m_reg, &ct::Registration::CorrespondenceRejectorDistance);
    connect(this, &Registration::CorrespondenceRejectorMedianDistance, m_reg, &ct::Registration::CorrespondenceRejectorMedianDistance);
    connect(this, &Registration::CorrespondenceRejectorOneToOne, m_reg, &ct::Registration::CorrespondenceRejectorOneToOne);
    connect(this, &Registration::CorrespondenceRejectionOrganizedBoundary, m_reg, &ct::Registration::CorrespondenceRejectionOrganizedBoundary);
    connect(this, &Registration::CorrespondenceRejectorPoly, m_reg, &ct::Registration::CorrespondenceRejectorPoly);
    connect(this, &Registration::CorrespondenceRejectorSampleConsensus, m_reg, &ct::Registration::CorrespondenceRejectorSampleConsensus);
    connect(this, &Registration::CorrespondenceRejectorSurfaceNormal, m_reg, &ct::Registration::CorrespondenceRejectorSurfaceNormal);
    connect(this, &Registration::CorrespondenceRejectorTrimmed, m_reg, &ct::Registration::CorrespondenceRejectorTrimmed);
    connect(this, &Registration::CorrespondenceRejectorVarTrimmed, m_reg, &ct::Registration::CorrespondenceRejectorVarTrimmed);
    connect(this, &Registration::TransformationEstimation2D, m_reg, &ct::Registration::TransformationEstimation2D);
    connect(this, &Registration::TransformationEstimation3Point, m_reg, &ct::Registration::TransformationEstimation3Point);
    connect(this, &Registration::TransformationEstimationDualQuaternion, m_reg, &ct::Registration::TransformationEstimationDualQuaternion);
    connect(this, &Registration::TransformationEstimationLM, m_reg, &ct::Registration::TransformationEstimationLM);
    connect(this, &Registration::TransformationEstimationPointToPlane, m_reg, &ct::Registration::TransformationEstimationPointToPlane);
    connect(this, &Registration::TransformationEstimationPointToPlaneLLS, m_reg, &ct::Registration::TransformationEstimationPointToPlaneLLS);
    connect(this, &Registration::TransformationEstimationSVD, m_reg, &ct::Registration::TransformationEstimationSVD);
    connect(this, &Registration::TransformationEstimationSymmetricPointToPlaneLLS, m_reg, &ct::Registration::TransformationEstimationSymmetricPointToPlaneLLS);
    connect(this, &Registration::IterativeClosestPoint, m_reg, &ct::Registration::IterativeClosestPoint);
    connect(this, &Registration::IterativeClosestPointWithNormals, m_reg, &ct::Registration::IterativeClosestPointWithNormals);
    connect(this, &Registration::IterativeClosestPointNonLinear, m_reg, &ct::Registration::IterativeClosestPointNonLinear);
    connect(this, &Registration::GeneralizedIterativeClosestPoint, m_reg, &ct::Registration::GeneralizedIterativeClosestPoint);
    connect(this, &Registration::NormalDistributionsTransform, m_reg, &ct::Registration::NormalDistributionsTransform);
    connect(this, &Registration::FPCSInitialAlignment, m_reg, &ct::Registration::FPCSInitialAlignment);
    connect(this, &Registration::KFPCSInitialAlignment, m_reg, &ct::Registration::KFPCSInitialAlignment);
    connect(m_reg, &ct::Registration::correspondenceEstimationResult, this, &Registration::correspondenceEstimationResult);
    connect(m_reg, &ct::Registration::correspondenceRejectorResult, this, &Registration::correspondenceRejectorResult);
    connect(m_reg, &ct::Registration::transformationEstimationResult, this, &Registration::transformationEstimationResult);
    connect(m_reg, &ct::Registration::registrationResult, this, &Registration::registrationResult);
    m_thread.start();

    connect(ui->cbox_TransEst, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int index)
            {
                if (index != REG_TYPE_TE_SymmetricPointToPlaneLLS)
                    ui->stackedWidget_5->setCurrentIndex(0);
                else
                    ui->stackedWidget_5->setCurrentIndex(1);
            });

    ui->cbox_type->setCurrentIndex(0);
    ui->cbox_feature->setCurrentIndex(0);
    ui->cbox_correEst->setCurrentIndex(0);
    ui->cbox_correRej->setCurrentIndex(0);
    ui->cbox_TransEst->setCurrentIndex(0);

    ui->stackedWidget->setCurrentIndex(0);
    ui->stackedWidget_2->setCurrentIndex(0);
    ui->stackedWidget_3->setCurrentIndex(0);
    ui->stackedWidget_4->setCurrentIndex(0);
    ui->stackedWidget_5->setCurrentIndex(0);
}

Registration::~Registration()
{
    m_thread.quit();
    m_thread.wait();
    delete ui;
}

void Registration::setTarget()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    m_target_cloud = selected_clouds.front();
    if (m_source_cloud && m_source_cloud->id() == m_target_cloud->id())
    {
        printW("Please choose another cloud as target cloud!");
        return;
    }
    m_cloudview->removeShape(m_target_cloud->boxId());
    m_cloudview->setPointCloudColor(m_target_cloud, QColorConstants::Red);
    m_cloudview->showInfo("Target Cloud(Red): " + m_target_cloud->id(), 1);
    if (m_source_cloud)
    {
        ui->btn_setTarget->setEnabled(false);
        ui->btn_setSource->setEnabled(false);
        m_cloudtree->setEnabled(false);
    }
}

void Registration::setSource()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    m_source_cloud = selected_clouds.front();
    if (m_target_cloud && m_target_cloud->id() == m_source_cloud->id())
    {
        printW("Please choose another cloud as target cloud!");
        return;
    }
    m_cloudview->removeShape(m_source_cloud->boxId());
    m_cloudview->setPointCloudColor(m_source_cloud, QColorConstants::Green);
    m_cloudview->showInfo("Source Cloud(Green): " + m_source_cloud->id(), 2);
    if (m_target_cloud)
    {
        ui->btn_setTarget->setEnabled(false);
        ui->btn_setSource->setEnabled(false);
        m_cloudtree->setEnabled(false);
    }
}

void Registration::setCorrespondenceEstimation()
{
    if (m_ce == nullptr)
    {
        printW("Please estimate correspondence first!");
        return;
    }
    m_reg->setCorrespondenceEstimation(m_ce);
    printI("Set CorrespondenceEstimation done!");
}

void Registration::addCorrespondenceRejector()
{
    if (m_cr_map.find(ui->cbox_correRej->currentIndex()) == m_cr_map.end())
    {
        printW("Please set CorrespondenceRejector first!");
        return;
    }
    m_reg->addCorrespondenceRejector(ui->cbox_correRej->currentIndex(), m_cr_map.find(ui->cbox_correRej->currentIndex())->second);
    printI("Add CorrespondenceRejector done!");
}

void Registration::removeCorrespondenceRejector()
{
    m_reg->removeCorrespondenceRejector(ui->cbox_correRej->currentIndex());
    printI("Remove CorrespondenceRejector done!");
}

void Registration::setTransformationEstimation()
{
    if (m_te == nullptr)
    {
        printW("Please estimate correspondence first!");
        return;
    }
    m_reg->setTransformationEstimation(m_te);
    printI("Set TransformationEstimation done!");
}

void Registration::preview()
{
    if (!m_source_cloud || !m_target_cloud)
    {
        printW("Please select a source cloud and a target cloud first!");
        return;
    }
    m_reg->setInputSource(m_source_cloud);
    m_reg->setInputTarget(m_target_cloud);
    m_cloudtree->showProgressBar();
    switch (ui->cbox_type->currentIndex())
    {
    case REG_TYPE_CorrespondenceEstimation:
        m_cloudview->removeCorrespondences(m_target_cloud->id() + m_source_cloud->id() + REG_CORRE_PRE_FLAG);
        switch (ui->cbox_correEst->currentIndex())
        {
        case REG_TYPE_CE_Base:
            m_cloudview->showInfo("CorrespondenceEstimation: Base", 3);
            if (ui->cbox_ce_type->currentIndex() > 0 && m_descriptor == nullptr)
            {
                printW("Please estimate target and source cloud descriptor first!");
                m_cloudtree->closeProgressBar();
                return;
            }
            switch (ui->cbox_ce_type->currentIndex())
            {
            case REG_TYPE_PointCloud:
                m_cloudview->showInfo("Estimation Type: PointCloud", 4);
                m_reg->CorrespondenceEstimation<ct::PointXYZRGBN>(m_source_cloud, m_target_cloud);
                break;
            case REG_TYPE_PFHFeature:
                m_cloudview->showInfo("Estimation Type: PFHFeature", 4);
                if (m_descriptor->getDescriptor(m_source_cloud->id())->pfh && m_descriptor->getDescriptor(m_target_cloud->id())->pfh)
                    m_reg->CorrespondenceEstimation<pcl::PFHSignature125>(m_descriptor->getDescriptor(m_source_cloud->id())->pfh,
                                                                          m_descriptor->getDescriptor(m_target_cloud->id())->pfh);
                else
                {
                    printW("Please estimate target and source cloud PFHFeature first!");
                    m_cloudtree->closeProgressBar();
                    return;
                }

                break;
            case REG_TYPE_FPFHFeature:
                m_cloudview->showInfo("Estimation Type: FPFHFeature", 4);
                if (m_descriptor->getDescriptor(m_source_cloud->id())->fpfh && m_descriptor->getDescriptor(m_target_cloud->id())->fpfh)
                    m_reg->CorrespondenceEstimation<pcl::FPFHSignature33>(m_descriptor->getDescriptor(m_source_cloud->id())->fpfh,
                                                                          m_descriptor->getDescriptor(m_target_cloud->id())->fpfh);
                else
                {
                    printW("Please estimate target and source cloud FPFHFeature first!");
                    m_cloudtree->closeProgressBar();
                    return;
                }
                break;
                //TODO: other feature
            }
            break;
        case REG_TYPE_CE_BackProjection:
            m_cloudview->showInfo("CorrespondenceEstimation: BackProjection", 3);
            if (!m_source_cloud->hasNormals() || !m_target_cloud->hasNormals())
            {
                printW("Please estimate target and source cloud normals first!");
                m_cloudtree->closeProgressBar();
                return;
            }
            emit CorrespondenceEstimationBackProjection(ui->spin_k1->value());
            break;
        case REG_TYPE_CE_NormalShooting:
            m_cloudview->showInfo("CorrespondenceEstimation: NormalShooting", 3);
            if (!m_target_cloud->hasNormals())
            {
                printW("Please estimate target cloud normals first!");
                m_cloudtree->closeProgressBar();
                return;
            }
            emit CorrespondenceEstimationNormalShooting(ui->spin_k2_2->value());
            break;
        }
        break;
    case REG_TYPE_CorrespondenceRejector:
        m_cloudview->removeCorrespondences(m_target_cloud->id() + m_source_cloud->id() + REG_CORRE_PRE_FLAG);
        if (m_corr == nullptr)
        {
            printW("please estimate target and source cloud correspondence first!");
            m_cloudtree->closeProgressBar();
            return;
        }
        m_reg->setInputCorrespondences(m_corr);
        switch (ui->cbox_correRej->currentIndex())
        {
        case REG_TYPE_CR_Distance:
            m_cloudview->showInfo("CorrespondenceRejector: Distance", 3);
            emit CorrespondenceRejectorDistance(ui->dspin_distance->value());
            break;
        case REG_TYPE_CR_MedianDistance:
            m_cloudview->showInfo("CorrespondenceRejector: MedianDistance", 3);
            emit CorrespondenceRejectorMedianDistance(ui->dspin_factor->value());
            break;
        case REG_TYPE_CR_OneToOne:
            m_cloudview->showInfo("CorrespondenceRejector: OneToOne", 3);
            emit CorrespondenceRejectorOneToOne();
            break;
        case REG_TYPE_CR_OrganizedBoundary:
            m_cloudview->showInfo("CorrespondenceRejector: OrganizedBoundary", 3);
            emit CorrespondenceRejectionOrganizedBoundary(ui->spin_val->value());
            break;
        case REG_TYPE_CR_Poly:
            m_cloudview->showInfo("CorrespondenceRejector: Poly", 3);
            emit CorrespondenceRejectorPoly(ui->spin_cardinality->value(), ui->dspin_similarity_threshold->value(),
                                            ui->spin_iterations->value());
            break;
        case REG_TYPE_CR_SampleConsensus:
            m_cloudview->showInfo("CorrespondenceRejector: SampleConsensus", 3);
            emit CorrespondenceRejectorSampleConsensus(ui->dspin_threshold->value(), ui->spin_max_iterations->value(),
                                                       ui->check_refine->isChecked());
            break;
        case REG_TYPE_CR_SurfaceNormal:
            m_cloudview->showInfo("CorrespondenceRejector: SurfaceNormal", 3);
            emit CorrespondenceRejectorSurfaceNormal(ui->dspin_threshold_2->value());
            break;
        case REG_TYPE_CR_Trimmed:
            m_cloudview->showInfo("CorrespondenceRejector: Trimmed", 3);
            emit CorrespondenceRejectorTrimmed(ui->dspin_ratio->value(), ui->spin_min_corre->value());
            break;
        case REG_TYPE_CR_VarTrimmed:
            m_cloudview->showInfo("CorrespondenceRejector: VarTrimmed", 3);
            emit CorrespondenceRejectorVarTrimmed(ui->dspin_min_ratio->value(), ui->dspin_max_ratio->value());
            break;
        }
        break;
    case REG_TYPE_TransformationEstimation:
        m_cloudview->removePointCloud(m_source_cloud->id() + REG_TRANS_PRE_FLAG);
        switch (ui->cbox_TransEst->currentIndex())
        {
        case REG_TYPE_TE_2D:
            emit TransformationEstimation2D();
            break;
        case REG_TYPE_TE_3Point:
            if ((m_target_cloud->size() != 3) || (m_source_cloud->size() != 3))
            {
                printW(QString("Number of points in source (%1) and target (%2) must be 3!").arg(m_source_cloud->size()).arg(m_target_cloud->size()));
                m_cloudtree->closeProgressBar();
                return;
            }
            emit TransformationEstimation3Point();
            break;
        case REG_TYPE_TE_DualQuaternion:
            emit TransformationEstimationDualQuaternion();
            break;
        case REG_TYPE_TE_LM:
            emit TransformationEstimationLM();
            break;
        case REG_TYPE_TE_PointToPlane:
            emit TransformationEstimationPointToPlane();
            break;
        case REG_TYPE_TE_PointToPlaneLLS:
            emit TransformationEstimationPointToPlaneLLS();
            break;
        case REG_TYPE_TE_SVD:
            emit TransformationEstimationSVD();
            break;
        case REG_TYPE_TE_SymmetricPointToPlaneLLS:
            emit TransformationEstimationSymmetricPointToPlaneLLS(ui->check_enforce_same_direction_normals->isChecked());
            break;
        }
        break;
    case REG_TYPE_Registration:
        m_cloudview->removePointCloud(m_source_cloud->id() + REG_ALIGN_PRE_FLAG);
        if (ui->check_continus->isChecked() && m_reg_map.find(m_source_cloud->id() + m_target_cloud->id()) != m_reg_map.end())
        {
            ct::Cloud::Ptr new_source_cloud = m_reg_map.find(m_source_cloud->id() + m_target_cloud->id())->second;
            m_reg->setInputSource(new_source_cloud);
        }
        m_reg->setMaximumIterations(ui->spin_nr_iterations->value());
        m_reg->setRANSACIterations(ui->spin_ransac_iterations->value());
        m_reg->setRANSACOutlierRejectionThreshold(ui->dspin_inlier_threshold->value());
        m_reg->setMaxCorrespondenceDistance(ui->dspin_distance_threshold->value());
        m_reg->setTransformationEpsilon(ui->dspin_transformation_epsilon->value());
        m_reg->setTransformationRotationEpsilon(ui->dspin_transformation_rotation_epsilon->value());
        m_reg->setEuclideanFitnessEpsilon(ui->dspin_euclidean_fitness_epsilon->value());
        switch (ui->cbox_registration->currentIndex())
        {
        case REG_IterativeClosestPoint:
            m_cloudview->showInfo("Registration: IterativeClosestPoint", 3);
            emit IterativeClosestPoint(ui->check_use_reciprocal->isChecked());
            break;
        case REG_IterativeClosestPointWithNormals:
            m_cloudview->showInfo("Registration: IterativeClosestPointWithNormals", 3);
            emit IterativeClosestPointWithNormals(ui->check_use_reciprocal_2->isChecked(), ui->check_use_symmetric_2->isChecked(),
                                                  ui->check_enforce_samedirection_2->isChecked());
            break;
        case REG_IterativeClosestPointNonLinear:
            m_cloudview->showInfo("Registration: IterativeClosestPointNonLinear", 3);
            emit IterativeClosestPointNonLinear(ui->check_use_reciprocal_3->isChecked());
            break;
        case REG_GeneralizedIterativeClosestPoint:
            m_cloudview->showInfo("Registration: GeneralizedIterativeClosestPoint", 3);
            emit GeneralizedIterativeClosestPoint(ui->spin_k_2->value(), ui->spin_max->value(), ui->dspin_tra_tolerance->value(),
                                                  ui->dspin_rol_tolerance->value(), ui->check_use_recip_corre->isChecked());
            break;
        case REG_SampleConsensusInitialAlignment:
            if (ui->cbox_feature2->currentIndex() > 0 && m_descriptor == nullptr)
            {
                printW("Please estimate target and source cloud descriptor first!");
                m_cloudtree->closeProgressBar();
                return;
            }
            m_cloudview->showInfo("Registration: SampleConsensusInitialAlignment", 3);
            switch (ui->cbox_feature2->currentIndex())
            {
            case REG_TYPE_PointCloud:
                printW("Please estimate target and source cloud Feature first!");
                m_cloudtree->closeProgressBar();
                return;
            case REG_TYPE_PFHFeature:
                m_cloudview->showInfo("Estimation Type: PFHFeature", 4);
                if (m_descriptor->getDescriptor(m_source_cloud->id())->pfh && m_descriptor->getDescriptor(m_target_cloud->id())->pfh)
                    m_reg->SampleConsensusInitialAlignment<pcl::PFHSignature125>(m_descriptor->getDescriptor(m_source_cloud->id())->pfh,
                                                                                 m_descriptor->getDescriptor(m_target_cloud->id())->pfh,
                                                                                 ui->dspin_min_sample_distance->value(),
                                                                                 ui->spin_nr_samples->value(),
                                                                                 ui->spin_k2->value());
                else
                {
                    printW("Please estimate target and source cloud PFHFeature first!");
                    m_cloudtree->closeProgressBar();
                    return;
                }

                break;
            case REG_TYPE_FPFHFeature:
                m_cloudview->showInfo("Estimation Type: FPFHFeature", 4);
                if (m_descriptor->getDescriptor(m_source_cloud->id())->fpfh && m_descriptor->getDescriptor(m_target_cloud->id())->fpfh)
                    m_reg->SampleConsensusInitialAlignment<pcl::FPFHSignature33>(m_descriptor->getDescriptor(m_source_cloud->id())->fpfh,
                                                                                 m_descriptor->getDescriptor(m_target_cloud->id())->fpfh,
                                                                                 ui->dspin_min_sample_distance->value(),
                                                                                 ui->spin_nr_samples->value(),
                                                                                 ui->spin_k2->value());
                else
                {
                    printW("Please estimate target and source cloud FPFHFeature first!");
                    m_cloudtree->closeProgressBar();
                    return;
                }
                break;
                //TODO: other feature
            }
            break;
        case REG_SampleConsensusPrerejective:
            if (ui->cbox_feature2->currentIndex() > 0 && m_descriptor == nullptr)
            {
                printW("Please estimate target and source cloud descriptor first!");
                m_cloudtree->closeProgressBar();
                return;
            }
            m_cloudview->showInfo("Registration: SampleConsensusPrerejective", 3);
            switch (ui->cbox_feature2->currentIndex())
            {
            case REG_TYPE_PointCloud:
                printW("Please estimate target and source cloud Feature first!");
                m_cloudtree->closeProgressBar();
                return;
            case REG_TYPE_PFHFeature:
                m_cloudview->showInfo("Estimation Type: PFHFeature", 4);
                if (m_descriptor->getDescriptor(m_source_cloud->id())->pfh && m_descriptor->getDescriptor(m_target_cloud->id())->pfh)
                    m_reg->SampleConsensusPrerejective<pcl::PFHSignature125>(m_descriptor->getDescriptor(m_source_cloud->id())->pfh,
                                                                             m_descriptor->getDescriptor(m_target_cloud->id())->pfh,
                                                                             ui->spin_nr_samples_2->value(),
                                                                             ui->spin_k->value(),
                                                                             ui->dspin_similarity_threshold->value(),
                                                                             ui->dspin_inlier_fraction->value());
                else
                {
                    printW("Please estimate target and source cloud PFHFeature first!");
                    m_cloudtree->closeProgressBar();
                    return;
                }

                break;
            case REG_TYPE_FPFHFeature:
                m_cloudview->showInfo("Estimation Type: FPFHFeature", 4);
                if (m_descriptor->getDescriptor(m_source_cloud->id())->fpfh && m_descriptor->getDescriptor(m_target_cloud->id())->fpfh)
                    m_reg->SampleConsensusPrerejective<pcl::FPFHSignature33>(m_descriptor->getDescriptor(m_source_cloud->id())->fpfh,
                                                                             m_descriptor->getDescriptor(m_target_cloud->id())->fpfh,
                                                                             ui->spin_nr_samples_2->value(),
                                                                             ui->spin_k->value(),
                                                                             ui->dspin_similarity_threshold->value(),
                                                                             ui->dspin_inlier_fraction->value());
                else
                {
                    printW("Please estimate target and source cloud FPFHFeature first!");
                    m_cloudtree->closeProgressBar();
                    return;
                }
                break;
                //TODO: other feature
            }
            break;
        case REG_NormalDistributionsTransform:
            m_cloudview->showInfo("Registration: NormalDistributionsTransform", 3);
            emit NormalDistributionsTransform(ui->dspin_resolution->value(), ui->dspin_step_size->value(), ui->dspin_outlier_ratio->value());
            break;
        case REG_FPCSInitialAlignment:
            m_cloudview->showInfo("Registration: FPCSInitialAlignment", 3);
            emit FPCSInitialAlignment(ui->dspin_delta->value(), ui->check_normalize->isChecked(), ui->dspin_approx_overlap->value(),
                                      ui->dspin_score_threshold->value(), ui->spin_nr_samples_3->value(), ui->dspin_max_norm_diff->value(),
                                      ui->dspin_max_runtime->value());
            break;
        case REG_KFPCSInitialAlignment:
            m_cloudview->showInfo("Registration: KFPCSInitialAlignment", 3);
            emit KFPCSInitialAlignment(ui->spin_nr_samples_4->value(), ui->check_normalize_2->isChecked(), ui->dspin_approx_overlap_2->value(),
                                       ui->dspin_score_threshold_2->value(), ui->spin_nr_samples_4->value(), ui->dspin_max_norm_diff->value(),
                                       ui->dspin_max_runtime->value(), ui->dspin_upper_trl_boundary->value(), ui->dspin_lower_trl_boundary->value(),
                                       ui->dspin_lambda->value());
            break;
        }
        break;
    }
}

void Registration::add()
{
    if (!m_source_cloud || !m_target_cloud)
    {
        printW("Please select a source cloud and a target cloud first!");
        return;
    }
    if (m_reg_map.find(m_source_cloud->id() + m_target_cloud->id()) == m_reg_map.end())
    {
        printW(QString("Please registrate target cloud[id:%1] and source cloud[id:%2] first.").arg(m_target_cloud->id()).arg(m_source_cloud->id()));
        return;
    }
    ct::Cloud::Ptr new_cloud = m_reg_map.find(m_source_cloud->id() + m_target_cloud->id())->second;
    m_cloudview->removePointCloud(new_cloud->id());
    new_cloud->setId(REG_ALIGN_ADD_FLAG + m_source_cloud->id());
    m_cloudtree->appendCloud(m_source_cloud, new_cloud, true);
    m_reg_map.erase(m_source_cloud->id() + m_target_cloud->id());
    printI(QString("Add registrated cloud[id:%1] done.").arg(new_cloud->id()));
    m_cloudview->clearInfo();
}

void Registration::apply()
{
    if (!m_source_cloud || !m_target_cloud)
    {
        printW("Please select a source cloud and a target cloud first!");
        return;
    }
    if (m_reg_map.find(m_source_cloud->id() + m_target_cloud->id()) == m_reg_map.end())
    {
        printW(QString("Please registrate target cloud[id:%1] and source cloud[id:%2] first.").arg(m_target_cloud->id()).arg(m_source_cloud->id()));
        return;
    }
    ct::Cloud::Ptr new_cloud = m_reg_map.find(m_source_cloud->id() + m_target_cloud->id())->second;
    m_cloudview->removePointCloud(new_cloud->id());
    m_cloudtree->updateCloud(m_source_cloud, new_cloud);
    m_reg_map.erase(m_source_cloud->id() + m_target_cloud->id());
    printI(QString("Apply registrated cloud[id:%1] done.").arg(new_cloud->id()));
    m_cloudview->clearInfo();
}

void Registration::reset()
{
    m_cloudtree->setEnabled(true);
    ui->btn_setSource->setEnabled(true);
    ui->btn_setTarget->setEnabled(true);
    m_cloudview->clearInfo();
    if (m_target_cloud) m_cloudview->resetPointCloudColor(m_target_cloud);
    if (m_source_cloud) m_cloudview->resetPointCloudColor(m_source_cloud);
    if (m_source_cloud && m_target_cloud)
    {
        m_cloudview->removeCorrespondences(m_target_cloud->id() + m_source_cloud->id() + REG_CORRE_PRE_FLAG);
        m_cloudview->removePointCloud(m_source_cloud->id() + REG_TRANS_PRE_FLAG);
        m_cloudview->removePointCloud(m_source_cloud->id() + REG_ALIGN_PRE_FLAG);
    }
    m_target_cloud = nullptr;
    m_source_cloud = nullptr;
    m_corr = nullptr;
    m_ce = nullptr;
    m_te = nullptr;
    for (auto& cloud : m_reg_map)
        m_cloudview->removePointCloud(cloud.second->id());
    m_reg_map.clear();
}

void Registration::correspondenceEstimationResult(const ct::CorrespondencesPtr& corr, float time, const ct::CorreEst::Ptr& ce)
{
    m_cloudtree->closeProgressBar();
    printI(QString("Estimate target cloud[id:%1] and source cloud[id:%2] correspondences done, take time %3 ms.").arg(m_target_cloud->id()).arg(m_source_cloud->id()).arg(time));
    QString id = m_target_cloud->id() + m_source_cloud->id() + REG_CORRE_PRE_FLAG;
    m_cloudview->addCorrespondences(m_source_cloud, m_target_cloud, corr, id);
    m_corr = corr;
    m_ce = ce;
}

void Registration::correspondenceRejectorResult(const ct::CorrespondencesPtr& corr, float time, const ct::CorreRej::Ptr& cj)
{
    m_cloudtree->closeProgressBar();
    printI(QString("Reject target cloud[id:%1] and source cloud[id:%2] correspondences done, take time %3 ms.").arg(m_target_cloud->id()).arg(m_source_cloud->id()).arg(time));
    QString id = m_target_cloud->id() + m_source_cloud->id() + REG_CORRE_PRE_FLAG;
    m_cloudview->addCorrespondences(m_source_cloud, m_target_cloud, corr, id);
    m_cr_map[ui->cbox_correRej->currentIndex()] = cj;
}

void Registration::transformationEstimationResult(const Eigen::Matrix4f& matrix, float time, const ct::TransEst::Ptr& te)
{
    m_cloudtree->closeProgressBar();
    printI(QString("Estimate target cloud[id:%1] and source cloud[id:%2] transformation done, take time %3 ms.").arg(m_target_cloud->id()).arg(m_source_cloud->id()).arg(time));
    ct::Cloud::Ptr cloud(new ct::Cloud);
    pcl::transformPointCloud(*m_source_cloud, *cloud, matrix);
    cloud->setId(m_source_cloud->id() + REG_TRANS_PRE_FLAG);
    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudSize(cloud->id(), cloud->pointSize() + 2);
    m_cloudview->setPointCloudColor(cloud->id(), QColorConstants::Blue);
    m_te = te;
}

void Registration::registrationResult(bool success, const ct::Cloud::Ptr& ail_cloud, double score, const Eigen::Matrix4f& matrix, float time)
{
    m_cloudtree->closeProgressBar();
    if (!success)
    {
        printE(QString("Registrate target cloud[id:%1] and source cloud[id:%2] failed!").arg(m_target_cloud->id()).arg(m_source_cloud->id()));
        return;
    }
    printI(QString("Registrate target cloud[id:%1] and source cloud[id:%2] done, take time %3 ms.").arg(m_target_cloud->id()).arg(m_source_cloud->id()).arg(time));
    ail_cloud->setId(m_source_cloud->id() + REG_ALIGN_PRE_FLAG);
    m_cloudview->addPointCloud(ail_cloud);
    m_cloudview->setPointCloudSize(ail_cloud->id(), ail_cloud->pointSize() + 2);
    m_cloudview->setPointCloudColor(ail_cloud->id(), QColorConstants::Blue);
    ui->txt_matrix->clear();
    ui->txt_matrix->append(tr("Fitness Score: %1 ").arg(score));
    ui->txt_matrix->append(tr("Transformation Matrix:"));
    ui->txt_matrix->append(ct::getTransformationQString(matrix, 6));
    m_reg_map[m_source_cloud->id() + m_target_cloud->id()] = ail_cloud;
}
