/**
 * @file filters.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "filters.h"
#include "ui_filters.h"

#define FILTER_TYPE_PassThrough                     (0)
#define FILTER_TYPE_VoxelGrid                       (1)
#define FILTER_TYPE_StatisticalOutlierRemoval       (2)
#define FILTER_TYPE_RadiusOutlierRemoval            (3)
#define FILTER_TYPE_ConditionalRemoval              (4)
#define FILTER_TYPE_MovingLeastSquares              (5)
#define FILTER_TYPE_GridMinimum                     (7)
#define FILTER_TYPE_LocalMaximum                    (8)
#define FILTER_TYPE_ShadowPoints                    (9)

#define FILTER_PRE_FLAG                             "-filter"
#define FILTER_ADD_FLAG                             "filtered-"


Filters::Filters(QWidget* parent)
    : CustomDock(parent), ui(new Ui::Filters),
    m_thread(this)
{
    ui->setupUi(this);

    qRegisterMetaType<std::string>("std::string &");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<ct::ConditionBase::Ptr>("ConditionBase::Ptr &");
    qRegisterMetaType<ct::ConditionBase::Ptr>("ConditionBase::Ptr");

    connect(ui->btn_preview, &QPushButton::clicked, this, &Filters::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Filters::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Filters::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Filters::reset);

    m_filters = new ct::Filters;
    m_filters->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_filters, &QObject::deleteLater);
    connect(this, &Filters::passThrough, m_filters, &ct::Filters::PassThrough);
    connect(this, &Filters::voxelGrid, m_filters, &ct::Filters::VoxelGrid);
    connect(this, &Filters::approximateVoxelGrid, m_filters, &ct::Filters::ApproximateVoxelGrid);
    connect(this, &Filters::statisticalOutlierRemoval, m_filters, &ct::Filters::StatisticalOutlierRemoval);
    connect(this, &Filters::radiusOutlierRemoval, m_filters, &ct::Filters::RadiusOutlierRemoval);
    connect(this, &Filters::conditionalRemoval, m_filters, &ct::Filters::ConditionalRemoval);
    connect(this, &Filters::movingLeastSquares, m_filters, &ct::Filters::MovingLeastSquares);
    connect(this, &Filters::gridMinimum, m_filters, &ct::Filters::GridMinimum);
    connect(this, &Filters::localMaximum, m_filters, &ct::Filters::LocalMaximum);
    connect(this, &Filters::shadowPoints, m_filters, &ct::Filters::ShadowPoints);
    connect(m_filters, &ct::Filters::filterResult, this, &Filters::filterResult);
    m_thread.start();

    // ConditionalRemoval
    connect(ui->btn_add_con, &QPushButton::clicked, [=]
            {
                int row = ui->table_condition->rowCount();
                ui->table_condition->setRowCount(row + 1);

                if (row == 0)
                {
                    QComboBox* con = new QComboBox;
                    con->addItems(QStringList({ "And", "Or" }));
                    ui->table_condition->setCellWidget(row, 0, con);
                }
                else if (row == 1)
                {
                    QComboBox* con = (QComboBox*)ui->table_condition->cellWidget(0, 0);
                    if (con->currentText() == "And")
                    {
                        ui->table_condition->removeCellWidget(0, 0);
                        ui->table_condition->setItem(0, 0, new QTableWidgetItem("And"));
                        ui->table_condition->setItem(row, 0, new QTableWidgetItem("And"));
                    }
                    else
                    {
                        ui->table_condition->removeCellWidget(0, 0);
                        ui->table_condition->setItem(0, 0, new QTableWidgetItem("Or"));
                        ui->table_condition->setItem(row, 0, new QTableWidgetItem("Or"));
                    }
                }
                else
                {
                    if (ui->table_condition->item(0, 0)->text() == "And")
                        ui->table_condition->setItem(row, 0, new QTableWidgetItem("And"));
                    else
                        ui->table_condition->setItem(row, 0, new QTableWidgetItem("Or"));
                }
                QComboBox* com = new QComboBox;
                com->addItems(QStringList({ "x", "y", "z", "r", "g", "b" }));
                ui->table_condition->setCellWidget(row, 1, com);

                QComboBox* op = new QComboBox;
                op->addItems(QStringList({ ">", ">=", "<", "<=", "=" }));
                ui->table_condition->setCellWidget(row, 2, op);

                QDoubleSpinBox* value = new QDoubleSpinBox;
                value->setDecimals(3);
                value->setRange(-99999, 99999);
                ui->table_condition->setCellWidget(row, 3, value);
            });

    connect(ui->btn_clear_con, &QPushButton::clicked, [=]
            {
                int row = ui->table_condition->rowCount();
                ui->table_condition->removeRow(row - 1);
            });

    //PassThrough
    connect(ui->dspin_min, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                ui->slider_min->setValue(value * 1000);
            });
    connect(ui->dspin_max, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                ui->slider_max->setValue(value * 1000);
            });
    connect(ui->slider_min, &QSlider::valueChanged, [=](int value)
            {
                ui->dspin_min->setValue((float)value / 1000);
                if (ui->check_refresh->isChecked())this->preview();
            });
    connect(ui->slider_max, &QSlider::valueChanged, [=](int value)
            {
                ui->dspin_max->setValue((float)value / 1000);
                if (ui->check_refresh->isChecked())this->preview();
            });
    //VoxelGrid
    connect(ui->check_same_value, &QCheckBox::stateChanged, [=](int state)
            {
                if (state)
                {
                    ui->dspin_leafy->setEnabled(false);
                    ui->dspin_leafz->setEnabled(false);
                }
                else
                {
                    ui->dspin_leafy->setEnabled(true);
                    ui->dspin_leafz->setEnabled(true);
                }
            });
    connect(ui->dspin_leafx, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->check_same_value->isChecked())
                {
                    ui->dspin_leafy->setValue(value);
                    ui->dspin_leafz->setValue(value);
                }
                if (ui->check_refresh->isChecked()) this->preview();
            });
    connect(ui->dspin_leafy, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
                if (!ui->check_same_value->isChecked() && ui->check_refresh->isChecked())this->preview();
            });
    connect(ui->dspin_leafz, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
                if (!ui->check_same_value->isChecked() && ui->check_refresh->isChecked())this->preview();
            });
    connect(ui->check_approximate, &QCheckBox::stateChanged, [=](int state)
            {
                if (state)
                    ui->check_reverse->setEnabled(false);
                else
                    ui->check_reverse->setEnabled(true);
            });
    //StatisticalOutlierRemoval
    connect(ui->spin_meank, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked()) this->preview();
            });
    connect(ui->dspin_stddevmulthresh, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked()) this->preview();
            });
    //RadiusOutlierRemoval
    connect(ui->dspin_radius, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked())this->preview();
            });
    connect(ui->spin_minneiborsinradius, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked())this->preview();
            });
    //MovingLeastSquares
    connect(ui->dspin_radius_2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked()) this->preview();
            });
    connect(ui->spin_polynomial_order, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked()) this->preview();
            });
    //GridMinimum
    connect(ui->dspin_resolution, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked()) this->preview();
            });
    //LocalMaximum
    connect(ui->dspin_radius_3, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked()) this->preview();
            });
    //ShadowPoints
    connect(ui->dspin_threshold, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked()) this->preview();
            });

    connect(ui->cbox_field_name, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Filters::getRange);

    ui->cbox_type->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
    ui->table_condition->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->check_refresh->setChecked(true);
}

Filters::~Filters()
{
    m_thread.quit();
    m_thread.wait();
    delete ui;
}

void Filters::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        m_filters->setInputCloud(cloud);
        m_filters->setNegative(ui->check_reverse->isChecked());
        switch (ui->cbox_type->currentIndex())
        {
        case FILTER_TYPE_PassThrough:
            m_cloudview->showInfo("PassThrough", 1);
            emit passThrough(ui->cbox_field_name->currentText().toStdString(), (float)ui->slider_min->value() / 1000, (float)ui->slider_max->value() / 1000);
            break;
        case FILTER_TYPE_VoxelGrid:
            m_cloudview->showInfo("VoxelGrid", 1);
            emit voxelGrid(ui->dspin_leafx->value(), ui->dspin_leafy->value(), ui->dspin_leafz->value());
            break;
        case FILTER_TYPE_StatisticalOutlierRemoval:
            m_cloudview->showInfo("StatisticalOutlierRemoval", 1);
            emit statisticalOutlierRemoval(ui->spin_meank->value(), ui->dspin_stddevmulthresh->value());
            break;
        case FILTER_TYPE_RadiusOutlierRemoval:
            m_cloudview->showInfo("RadiusOutlierRemoval", 1);
            emit radiusOutlierRemoval(ui->dspin_radius->value(), ui->spin_minneiborsinradius->value());
            break;
        case FILTER_TYPE_ConditionalRemoval:
            m_cloudview->showInfo("ConditionalRemoval", 1);
            emit conditionalRemoval(this->getCondition());
            break;
        case FILTER_TYPE_MovingLeastSquares:
            m_cloudview->showInfo("MovingLeastSquares", 1);
            emit movingLeastSquares(ui->check_compute_normals->isChecked(), ui->spin_polynomial_order->value(), ui->dspin_radius_2->value());
            break;
            break;
        case FILTER_TYPE_GridMinimum:
            m_cloudview->showInfo("GridMinimum", 1);
            emit gridMinimum(ui->dspin_resolution->value());
            break;
        case FILTER_TYPE_LocalMaximum:
            m_cloudview->showInfo("LocalMaximum", 1);
            emit localMaximum(ui->dspin_radius_3->value());
            break;
        case FILTER_TYPE_ShadowPoints:
            if (!cloud->hasNormals())
            {
                printW("Please estimate normals first!");
                return;
            }
            m_cloudview->showInfo("ShadowPoints", 1);
            emit shadowPoints(ui->dspin_threshold->value());
            break;
        }
        if (!ui->check_refresh->isChecked()) m_cloudtree->showProgressBar();
    }
}

void Filters::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_filter_map.find(cloud->id()) == m_filter_map.end())
        {
            printW(QString("The cloud[id:%1] has no filtered cloud !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_filter_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        new_cloud->setId(FILTER_ADD_FLAG + cloud->id());
        m_cloudtree->appendCloud(cloud, new_cloud, true);
        m_filter_map.erase(cloud->id());
        printI(QString("Add filtered cloud[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Filters::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_filter_map.find(cloud->id()) == m_filter_map.end())
        {
            printW(QString("The cloud[id:%1] has no filtered cloud !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_filter_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_filter_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply filtered cloud[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Filters::reset()
{
    for (auto& cloud : m_cloudtree->getSelectedClouds())
        m_cloudtree->setCloudChecked(cloud);
    for (auto& cloud : m_filter_map)
        m_cloudview->removePointCloud(cloud.second->id());
    m_filter_map.clear();
    m_cloudview->clearInfo();
}

void Filters::filterResult(const ct::Cloud::Ptr& cloud, float time)
{
    printI(QString("Filter cloud[id:%1] done, take time %2 ms.").arg(cloud->id()).arg(time));
    QString id = cloud->id();
    cloud->setId(id + FILTER_PRE_FLAG);
    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudColor(cloud->id(), QColorConstants::Green);
    m_cloudview->setPointCloudSize(cloud->id(), cloud->pointSize() + 2);
    m_filter_map[id] = cloud;
    if (!ui->check_refresh->isChecked()) m_cloudtree->closeProgressBar();
}


ct::ConditionBase::Ptr Filters::getCondition()
{
    int rowCount = ui->table_condition->rowCount();
    QString condition;
    if (rowCount == 0)  return nullptr;
    else if (rowCount == 1)
        condition = ((QComboBox*)ui->table_condition->cellWidget(0, 0))->currentText();
    else
        condition = ui->table_condition->item(0, 0)->text();
    std::string  field;
    ct::CompareOp op;
    double value;
    if (condition == "And")
    {
        ct::ConditionAnd::Ptr and_cond(new ct::ConditionAnd);
        for (int i = 0; i < rowCount; i++)
        {
            field = ((QComboBox*)ui->table_condition->cellWidget(i, 1))->currentText().toStdString();
            op = ct::CompareOp(((QComboBox*)ui->table_condition->cellWidget(i, 2))->currentIndex());
            value = ((QDoubleSpinBox*)ui->table_condition->cellWidget(i, 3))->value();
            ct::FieldComparison::Ptr  filedcomp(new ct::FieldComparison(field, op, value));
            and_cond->addComparison(filedcomp);
        }
        return and_cond;
    }
    else
    {
        ct::ConditionOr::Ptr or_cond(new ct::ConditionOr);
        for (int i = 0; i < rowCount; i++)
        {
            field = ((QComboBox*)ui->table_condition->cellWidget(i, 1))->currentText().toStdString();
            op = ct::CompareOp(((QComboBox*)ui->table_condition->cellWidget(i, 2))->currentIndex());
            value = ((QDoubleSpinBox*)ui->table_condition->item(i, 3))->value();
            ct::FieldComparison::Ptr  filedcomp(new ct::FieldComparison(field, op, value));
            or_cond->addComparison(filedcomp);
        }
        return or_cond;
    }
}

void Filters::getRange(int index)
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    float min = std::numeric_limits<float>::max(), max = -std::numeric_limits<float>::max();
    for (auto& cloud : selected_clouds)
    {
        switch (index)
        {
        case 0://x
            min = cloud->min().x < min ? cloud->min().x : min;
            max = cloud->max().x > min ? cloud->max().x : max;
            break;
        case 1://y
            min = cloud->min().y < min ? cloud->min().y : min;
            max = cloud->max().y > min ? cloud->max().y : max;
            break;
        case 2://z
            min = cloud->min().z < min ? cloud->min().z : min;
            max = cloud->max().z > min ? cloud->max().z : max;
            break;
        case 3://rgb
            min = 0, max = 1;
            break;
        case 4://curvature
            min = 0, max = 1;
            break;
        }
        min *= 1000, max *= 1000;
        ui->slider_min->setRange(min, max);
        ui->slider_max->setRange(min, max);
        ui->slider_min->setValue(min);
        ui->slider_max->setValue(max);
    }
}