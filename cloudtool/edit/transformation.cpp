/**
 * @file transformation.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "transformation.h"

#include "base/common.h"
#include "ui_transformation.h"

#define TRANS_TYPE_MATRIX       (0)
#define TRANS_TYPE_EULERANGLE   (1)
#define TRANS_TYPE_ANGLEAXIS    (2)
#define TRANS_PRE_FLAG          "-trans"
#define TRANS_ADD_FLAG          "transed-"

Transformation::Transformation(QWidget* parent)
    : CustomDock(parent), ui(new Ui::Transformation),
    m_affine(Eigen::Affine3f::Identity())
{
    ui->setupUi(this);
    connect(ui->btn_add, &QPushButton::clicked, this, &Transformation::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Transformation::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Transformation::reset);
    connect(ui->btn_preview, &QPushButton::clicked, [=] { this->preview(m_affine); });
    ui->tabWidget->setCurrentIndex(0);

    // matrix
    connect(ui->txt_matrix, &QTextEdit::textChanged, [=]
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_MATRIX) return;
                if (!ct::getTransformation(ui->txt_matrix->toPlainText(), m_affine))
                {
                    printW("The transformation matrix format is wrong!");
                    return;
                }
                emit affine(m_affine);
            });

    // eulerAngle
    connect(ui->dspin_rx1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE)  return;
                m_affine = ct::getTransformation(ui->dspin_tx1->value(), ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                                                 value, ui->dspin_ry1->value(), ui->dspin_rz1->value());
                emit affine(m_affine);
            });
    connect(ui->dspin_ry1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE)  return;
                m_affine = ct::getTransformation(ui->dspin_tx1->value(), ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                                                 ui->dspin_rx1->value(), value, ui->dspin_rz1->value());
                emit affine(m_affine);
            });
    connect(ui->dspin_rz1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE)  return;
                m_affine = ct::getTransformation(ui->dspin_tx1->value(), ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                                                 ui->dspin_rx1->value(), ui->dspin_ry1->value(), value);
                emit affine(m_affine);
            });
    connect(ui->dspin_tx1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE)  return;
                m_affine = ct::getTransformation(value, ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                                                 ui->dspin_rx1->value(), ui->dspin_ry1->value(), ui->dspin_rz1->value());
                emit affine(m_affine);
            });
    connect(ui->dspin_ty1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE)  return;
                m_affine = ct::getTransformation(ui->dspin_tx1->value(), value, ui->dspin_tz1->value(),
                                                 ui->dspin_rx1->value(), ui->dspin_ry1->value(), ui->dspin_rz1->value());
                emit affine(m_affine);
            });
    connect(ui->dspin_tz1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE)  return;
                m_affine = ct::getTransformation(ui->dspin_tx1->value(), ui->dspin_ty1->value(), value,
                                                 ui->dspin_rx1->value(), ui->dspin_ry1->value(), ui->dspin_rz1->value());
                emit affine(m_affine);
            });
    connect(ui->txt_xyzeuler, &QLineEdit::textChanged, [=](const QString& text)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE) return;
                if (!ct::getTransformation(text, m_affine))
                {
                    printW("The transformation xyzeuler format is wrong!");
                    return;
                }
                emit affine(m_affine);
            });

    // axisAngle
    connect(ui->dspin_angle, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = ct::getTransformation(value, ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                emit affine(m_affine);
            });
    connect(ui->dspin_ax, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = ct::getTransformation(ui->dspin_angle->value(), value, ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                emit affine(m_affine);
            });
    connect(ui->dspin_ay, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = ct::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), value, ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                emit affine(m_affine);
            });
    connect(ui->dspin_az, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = ct::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), value,
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                emit affine(m_affine);
            });
    connect(ui->dspin_tx2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = ct::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 value, ui->dspin_ty2->value(), ui->dspin_tz2->value());
                emit affine(m_affine);
            });
    connect(ui->dspin_ty2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = ct::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), value, ui->dspin_tz2->value());
                emit affine(m_affine);
            });
    connect(ui->dspin_tz2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = ct::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), value);
                emit affine(m_affine);
            });

    // singal
    connect(this, &Transformation::affine, [=](const Eigen::Affine3f& affine3f)
            {
                float x, y, z, rx, ry, rz;
                ct::getTranslationAndEulerAngles(affine3f, x, y, z, rx, ry, rz);
                float alpha, axisX, axisY, axisZ;
                ct::getAngleAxis(affine3f, alpha, axisX, axisY, axisZ);
                int index = ui->tabWidget->currentIndex();
                if (index != TRANS_TYPE_MATRIX)
                    ui->txt_matrix->setText(ct::getTransformationQString(affine3f.matrix(), 3));
                if (index != TRANS_TYPE_EULERANGLE)
                {
                    ui->dspin_rx1->setValue(rx);
                    ui->dspin_ry1->setValue(ry);
                    ui->dspin_rz1->setValue(rz);
                    ui->dspin_tx1->setValue(x);
                    ui->dspin_ty1->setValue(y);
                    ui->dspin_tz1->setValue(z);
                }
                if (index != TRANS_TYPE_ANGLEAXIS)
                {
                    ui->dspin_angle->setValue(alpha);
                    ui->dspin_ax->setValue(axisX);
                    ui->dspin_ay->setValue(axisY);
                    ui->dspin_az->setValue(axisZ);
                    ui->dspin_tx2->setValue(x);
                    ui->dspin_ty2->setValue(y);
                    ui->dspin_tz2->setValue(z);
                }
            });
    connect(this, &Transformation::affine, this, &Transformation::preview);
}

Transformation::~Transformation() { delete ui; }

void Transformation::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (m_affine.matrix() == Eigen::Matrix4f::Identity()) return;
    for (auto& cloud : selected_clouds)
    {
        if (m_trans_map.find(cloud->id()) == m_trans_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched transformation !").arg(cloud->id()));
            continue;
        }
        m_cloudview->removePointCloud(cloud->id() + TRANS_PRE_FLAG);
        ct::Cloud::Ptr new_cloud = cloud->makeShared();
        pcl::transformPointCloud(*cloud, *new_cloud, m_trans_map.find(cloud->id())->second);
        new_cloud->setId(TRANS_ADD_FLAG + cloud->id());
        m_cloudtree->appendCloud(cloud, new_cloud, true);
        m_trans_map.erase(cloud->id());
        printI(QString("Add transformed cloud[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Transformation::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (m_affine.matrix() == Eigen::Matrix4f::Identity()) return;
    for (auto& cloud : selected_clouds)
    {
        if (m_trans_map.find(cloud->id()) == m_trans_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched transformation !").arg(cloud->id()));
            continue;
        }
        m_cloudview->removePointCloud(cloud->id() + TRANS_PRE_FLAG);
        pcl::transformPointCloud(*cloud, *cloud, m_trans_map.find(cloud->id())->second);
        m_cloudtree->updateCloud(cloud, cloud);
        m_trans_map.erase(cloud->id());
        printI(QString("Apply transformed cloud[id:%1] done.").arg(cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Transformation::reset()
{
    for (auto& cloud : m_trans_map)
        m_cloudview->removePointCloud(cloud.first + TRANS_PRE_FLAG);
    m_trans_map.clear();
    m_cloudview->clearInfo();
}

void Transformation::preview(const Eigen::Affine3f& affine3f)
{
    m_affine = affine3f;
    if (!ui->check_refresh->isChecked()) return;
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (m_affine.matrix() == Eigen::Matrix4f::Identity()) return;
    m_cloudview->showInfo("Transform Pointcloud", 1);
    for (auto& cloud : selected_clouds)
    {
        ct::Cloud::Ptr trans_cloud = cloud->makeShared();
        trans_cloud->setId(cloud->id() + TRANS_PRE_FLAG);
        m_cloudview->addPointCloud(trans_cloud);
        if (!ui->cbox_inverse->isChecked())
        {
            m_cloudview->updateCloudPose(trans_cloud->id(), m_affine);
            m_trans_map[cloud->id()] = m_affine;
        }
        else
        {
            m_cloudview->updateCloudPose(trans_cloud->id(), m_affine.inverse());
            m_trans_map[cloud->id()] = m_affine.inverse();
        }
        m_cloudview->setPointCloudSize(trans_cloud->id(), 3);
    }
}

