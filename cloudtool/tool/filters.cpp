/**
 * @file filters.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "filters.h"

#include <QComboBox>
#include <QDebug>

#include "ui_filters.h"

#define FILTER_TYPE_PassThrough                     (0)
#define FILTER_TYPE_VoxelGrid                       (1)
#define FILTER_TYPE_StatisticalOutlierRemoval       (2)
#define FILTER_TYPE_RadiusOutlierRemoval            (3)
#define FILTER_TYPE_ConditionalRemoval              (4)
#define FILTER_TYPE_MovingLeastSquares              (5)
#define FILTER_TYPE_GridMinimum                     (6)
#define FILTER_TYPE_LocalMaximum                    (7)
#define FILTER_TYPE_ShadowPoints                    (8)

#define FILTER_PRE_FLAG                             "-filter"
#define FILTER_ADD_FLAG                             "filtered-"


Filters::Filters(QWidget* parent) : CustomDock(parent), ui(new Ui::Filters), m_thread(this)
{
    ui->setupUi(this);

    qRegisterMetaType<ct::Condition::Ptr>("ct::Condition::Ptr &");
    qRegisterMetaType<ct::Condition::Ptr>("ct::Condition::Ptr");

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
    m_filters.start();

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
                ui->table_condition->setCellWidget(row, 3, value);
            });

    connect(ui->btn_clear_con, &QPushButton::clicked, [=]
            {
                int row = ui->table_condition->rowCount();
                ui->table_condition->removeRow(row - 1);
            });

    //PassThrough
    connect(ui->dspin_min, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked())this->preview();
            });
    connect(ui->dspin_max, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
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
    //RadiusOutlierRemoval
    connect(ui->dspin_radius_2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked()) this->preview();
            });
    connect(ui->spin_polynomial_order, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=]
            {
                if (ui->check_refresh->isChecked()) this->preview();
            });

    connect(ui->cbox_field_name, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Filters::getRange);

    ui->cbox_type->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
    ui->table_condition->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

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
            emit passThrough(ui->cbox_field_name->currentText().toStdString(), ui->dspin_min->value(), ui->dspin_max->value());
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
        case FILTER_TYPE_GridMinimum:
            m_cloudview->showInfo("GridMinimum", 1);
            emit gridMinimum(ui->dspin_resolution->value());
            break;
        case FILTER_TYPE_LocalMaximum:
            m_cloudview->showInfo("LocalMaximum", 1);
            emit localMaximum(ui->dspin_radius_3->value());
            break;
        case FILTER_TYPE_ShadowPoints:
            m_cloudview->showInfo("ShadowPoints", 1);
            emit shadowPoints(ui->dspin_threshold->value());
            break;
        }
        if (!ui->check_refresh->isChecked()) m_cloudtree->showProgressBar();
    }
}

void Filters::add()
{
    //    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    //    if(selectedClouds.empty()) {
    //        console->warning(tr("Please select a pointcloud!"));
    //        return;
    //    }
    //    if(filtered_clouds.empty()||selectedClouds.size()!=filtered_clouds.size()){
    //        console->warning(tr("Please filter the pointcloud first!"));
    //        return;
    //    }
    //    cloud_view->removeShape("info");
    //    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    //    for(size_t i=0;i<selectedClouds.size();i++) {
    //        cloud_view->removeCloud(selectedClouds[i]->id+"-filtered");
    //        filtered_clouds[i]->prefix("filtered-");
    //        filtered_clouds[i]->update();
    //        cloud_tree->insertCloud(indexs[i].row,filtered_clouds[i],true);
    //        cloud_view->updateCube(filtered_clouds[i]->box,filtered_clouds[i]->box_id);
    //    }
    //    console->info(tr("Added successfully!"));
    //    filtered_clouds.clear();
}

void Filters::apply()
{
    //    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    //    if(selectedClouds.empty()) {
    //        console->warning(tr("Please select a pointcloud!"));
    //        return;
    //    }
    //    if(filtered_clouds.empty()||selectedClouds.size()!=filtered_clouds.size()){
    //        this->preview();
    //        return;
    //    }
    //    cloud_view->removeShape("info");
    //    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    //    for(size_t i=0;i<selectedClouds.size();i++) {
    //        cloud_tree->setCloudChecked(indexs[i],true);
    //        cloud_view->removeCloud(selectedClouds[i]->id+"-filtered");
    //        selectedClouds[i]->swap(*filtered_clouds[i]);
    //        selectedClouds[i]->update();
    //        cloud_view->updateCloud(selectedClouds[i],selectedClouds[i]->id);
    //        cloud_view->updateCube(selectedClouds[i]->box,selectedClouds[i]->box_id);
    //    }
    //    console->info(tr("Applied successfully!"));
    //    filtered_clouds.clear();
}

void Filters::reset()
{
    //    cloud_view->removeShape("info");
    //    filtered_clouds.clear();
    //    std::vector<Cloud::Ptr>selectedClouds=cloud_tree->getSelectedClouds();
    //    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    //    for(size_t i=0;i<selectedClouds.size();i++) {
    //        if(!cloud_view->contains(selectedClouds[i]->id))
    //            cloud_tree->setCloudChecked(indexs[i],true);
    //        cloud_view->removeCloud(selectedClouds[i]->id+"-filtered");
    //    }
}

void Filters::filterResult(const ct::Cloud::Ptr& cloud, float time)
{
    printI(QString("Filter cloud[id:%1] done, take time %2 ms.").arg(cloud->id()).arg(time));
    cloud->setId(cloud->id() + FILTER_PRE_FLAG);
    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudColor(cloud->id(), QColorConstants::Green);
    m_cloudview->setPointCloudSize(cloud->id(), cloud->pointSize() + 2);
    filtered_map[cloud->id()] = cloud;
    if (!ui->check_refresh->isChecked()) m_cloudtree->closeProgressBar();
}

// bool Filters::checkValid(bool preview)
// {
    // selected_clouds = cloudtree->getSelectedClouds();
    // if (selected_clouds.empty())
    // {
    //     log(pca::LOG_WARNING, "please select a pointcloud!");
    //     return false;
    // }
    // if (!preview)
    //     for (auto& cloud : selected_clouds)
    //         if (filtered_map.find(cloud->id()) == filtered_map.end())
    //         {
    //             log(pca::LOG_WARNING,
    //                 "please estimation the keypoints or select the correct "
    //                 "pointcloud!");
    //             return false;
    //         }
//     return true;
// }

ct::Condition::Ptr Filters::getCondition()
{
    // int rowCount = ui->table_condition->rowCount();
    // QString condition;
    // if (rowCount == 0)
    //     return nullptr;
    // else if (rowCount == 1)
    //     condition =
    //     ((QComboBox*)ui->table_condition->cellWidget(0, 0))->currentText();
    // else
    //     condition = ui->table_condition->item(0, 0)->text();
    // if (condition == "And")
    // {
    //     pcl::ConditionAnd<pca::PointXYZRGBN>::Ptr and_cond(
    //         new pcl::ConditionAnd<pca::PointXYZRGBN>());
    //     for (int i = 0; i < rowCount; i++)
    //     {
    //         and_cond->addComparison(pcl::FieldComparison<pca::PointXYZRGBN>::ConstPtr(
    //             new pcl::FieldComparison<pca::PointXYZRGBN>(
    //                 ((QComboBox*)ui->table_condition->cellWidget(i, 1))
    //                 ->currentText()
    //                 .toStdString(),
    //                 pcl::ComparisonOps::CompareOp(
    //                     ((QComboBox*)ui->table_condition->cellWidget(i, 2))
    //                     ->currentIndex()),
    //                 ui->table_condition->item(i, 3)->text().toFloat())));
    //     }
    //     return and_cond;
    // }
    // else
    // {
    pcl::ConditionOr<ct::PointXYZRGBN>::Ptr or_cond(new pcl::ConditionOr<ct::PointXYZRGBN>());
    // for (int i = 0; i < rowCount; i++)
    // {
    //     or_cond->addComparison(pcl::FieldComparison<pca::PointXYZRGBN>::ConstPtr(
    //         new pcl::FieldComparison<pca::PointXYZRGBN>(
    //             ((QComboBox*)ui->table_condition->cellWidget(i, 1))
    //             ->currentText()
    //             .toStdString(),
    //             pcl::ComparisonOps::CompareOp(
    //                 ((QComboBox*)ui->table_condition->cellWidget(i, 2))
    //                 ->currentIndex()),
    //             ui->table_condition->item(i, 3)->text().toFloat())));
    // }
    return or_cond;
    // }
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
    switch (index)
    {
    case 0://x
        for (auto& i : selected_clouds)
        {
            if (i->min.x < min) min = i->min.x;if (i->max.x > max) max = i->max.x;
        }
        ui->dspin_min->setRange(min, max);
        ui->dspin_max->setRange(min, max);
        ui->dspin_min->setValue(min);
        ui->dspin_max->setValue(max);
        break;
    case 1://y
        for (auto& i : selected_clouds)
        {
            if (i->min.y < min) min = i->min.y;if (i->max.y > max) max = i->max.y;
        }
        ui->dspin_min->setRange(min, max);
        ui->dspin_max->setRange(min, max);
        ui->dspin_min->setValue(min);
        ui->dspin_max->setValue(max);
        break;
    case 2://z
        for (auto& i : selected_clouds)
        {
            if (i->min.z < min) min = i->min.z;if (i->max.z > max) max = i->max.z;
        }
        ui->dspin_min->setRange(min, max);
        ui->dspin_max->setRange(min, max);
        ui->dspin_min->setValue(min);
        ui->dspin_max->setValue(max);
        break;
    case 3://rgb
        ui->dspin_min->setValue(0);
        ui->dspin_max->setValue(1);
        break;
    case 4://curvature
        ui->dspin_min->setValue(0);
        ui->dspin_max->setValue(1);
        break;
    }
}
