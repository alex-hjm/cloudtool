#include "filters.h"

#include <QComboBox>
#include <QDebug>

#include "ui_filters.h"

Filters::Filters(QWidget *parent) : CustomDock(parent), ui(new Ui::Filters) {
  ui->setupUi(this);

  qRegisterMetaType<pca::Condition::Ptr>("pca::Condition::Ptr &");
  qRegisterMetaType<pca::Condition::Ptr>("pca::Condition::Ptr");

  connect(ui->btn_preview, &QPushButton::clicked, this, &Filters::preview);
  connect(ui->btn_add, &QPushButton::clicked, this, &Filters::add);
  connect(ui->btn_apply, &QPushButton::clicked, this, &Filters::apply);
  connect(ui->btn_reset, &QPushButton::clicked, this, &Filters::reset);

  ui->table_condition->horizontalHeader()->setSectionResizeMode(
      QHeaderView::Stretch);

  filters = new pca::Filters;
  filters->moveToThread(&thread);
  connect(&thread, &QThread::finished, filters, &QObject::deleteLater);
  connect(this, &Filters::ConditionalRemoval, filters,
          &pca::Filters::ConditionalRemoval);
  connect(filters, &pca::Filters::filterResult, this, &Filters::filterResult);
  thread.start();

  // ConditionalRemoval
  connect(ui->btn_add_con, &QPushButton::clicked, [=] {
    int row = ui->table_condition->rowCount();
    ui->table_condition->setRowCount(row + 1);

    if (row == 0) {
      QComboBox *con = new QComboBox;
      con->addItems(QStringList({"And", "Or"}));
      ui->table_condition->setCellWidget(row, 0, con);
    } else if (row == 1) {
      QComboBox *con = (QComboBox *)ui->table_condition->cellWidget(0, 0);
      if (con->currentText() == "And") {
        ui->table_condition->removeCellWidget(0, 0);
        ui->table_condition->setItem(0, 0, new QTableWidgetItem("And"));
        ui->table_condition->setItem(row, 0, new QTableWidgetItem("And"));
      } else {
        ui->table_condition->removeCellWidget(0, 0);
        ui->table_condition->setItem(0, 0, new QTableWidgetItem("Or"));
        ui->table_condition->setItem(row, 0, new QTableWidgetItem("Or"));
      }
    } else {
      if (ui->table_condition->item(0, 0)->text() == "And")
        ui->table_condition->setItem(row, 0, new QTableWidgetItem("And"));
      else
        ui->table_condition->setItem(row, 0, new QTableWidgetItem("Or"));
    }
    QComboBox *com = new QComboBox;
    com->addItems(QStringList({"x", "y", "z", "r", "g", "b"}));
    ui->table_condition->setCellWidget(row, 1, com);

    QComboBox *op = new QComboBox;
    op->addItems(QStringList({"GT", "GE", "LT", "LE", "EQ"}));
    ui->table_condition->setCellWidget(row, 2, op);
    ui->table_condition->setItem(row, 3, new QTableWidgetItem("0"));
  });

  connect(ui->btn_clear_con, &QPushButton::clicked, [=] {
    int row = ui->table_condition->rowCount();
    ui->table_condition->removeRow(row - 1);
  });

  ui->cbox_type->setCurrentIndex(0);
  ui->stackedWidget->setCurrentIndex(0);

  // ui->dspin_user->setValue(std::numeric_limits<float>::quiet_NaN());
  // qDebug() << std::numeric_limits<float>::quiet_NaN();

  //	//PassThrough
  //    connect(ui->dspin_min_limit,static_cast<void
  //    (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
  //        if(ui->check_refresh->isChecked())
  //            this->preview();
  //    });
  //    connect(ui->dspin_max_limit,static_cast<void
  //    (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
  //        if(ui->check_refresh->isChecked())
  //            this->preview();
  //    });
  //    //VoxelGrid
  //    connect(ui->check_same_value,&QCheckBox::stateChanged,[=](int state){
  //        if(state){
  //            ui->dspin_leafy->setEnabled(false);
  //            ui->dspin_leafz->setEnabled(false);
  //        }
  //        else {
  //            ui->dspin_leafy->setEnabled(true);
  //            ui->dspin_leafz->setEnabled(true);
  //        }
  //    });
  //    connect(ui->dspin_leafx,static_cast<void
  //    (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double
  //    value){
  //        if(ui->check_same_value->isChecked()){
  //            ui->dspin_leafy->setValue(value);
  //            ui->dspin_leafz->setValue(value);
  //        }
  //        if(ui->check_refresh->isChecked())
  //            this->preview();
  //    });
  //    connect(ui->dspin_leafy,static_cast<void
  //    (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
  //        if(!ui->check_same_value->isChecked()&&ui->check_refresh->isChecked())
  //            this->preview();
  //    });
  //    connect(ui->dspin_leafz,static_cast<void
  //    (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
  //        if(!ui->check_same_value->isChecked()&&ui->check_refresh->isChecked())
  //            this->preview();
  //    });
  //    connect(ui->check_approximate,&QCheckBox::stateChanged,[=](int state){
  //        if(state)
  //            ui->check_reverse->setEnabled(false);
  //        else
  //            ui->check_reverse->setEnabled(true);
  //    });
  //    //StatisticalOutlierRemoval
  //    connect(ui->spin_meank,static_cast<void
  //    (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=]{
  //        if(ui->check_refresh->isChecked())
  //            this->preview();
  //    });
  //    connect(ui->dspin_stddevmulthresh,static_cast<void
  //    (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
  //        if(ui->check_refresh->isChecked())
  //            this->preview();
  //    });
  //    //RadiusOutlierRemoval
  //    connect(ui->dspin_radius,static_cast<void
  //    (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
  //        if(ui->check_refresh->isChecked())
  //            this->preview();
  //    });
  //    connect(ui->spin_minneiborsinradius,static_cast<void
  //    (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=]{
  //        if(ui->check_refresh->isChecked())
  //            this->preview();
  //    });
  //    //RadiusOutlierRemoval
  //    connect(ui->dspin_radius_2,static_cast<void
  //    (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
  //        if(ui->check_refresh->isChecked())
  //            this->preview();
  //    });
  //    connect(ui->spin_polynomialOrder,static_cast<void
  //    (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=]{
  //        if(ui->check_refresh->isChecked())
  //            this->preview();
  //    });
}

Filters::~Filters() {
  thread.quit();
  thread.wait();
  delete ui;
}

void Filters::init() {
  //    console=co;cloud_view=cv;cloud_tree=ct;
  //    //PassThrough
  //    connect(ui->cbox_fieid_name,static_cast<void
  //    (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
  //            this,&Filters::getRangeFromCloud);
  //    this->getRangeFromCloud(ui->cbox_fieid_name->currentIndex());
  //    ui->filtersWidget->setCurrentIndex(0);
  //    ui->cbox_filters->setCurrentIndex(0);
  //    ui->check_refresh->setChecked(true);
  //    connect(cloud_tree,&CloudTree::removedId,this,&Filters::removeCloud);
}

void Filters::preview() {
  if (!checkValid(true)) return;
  for (auto &cloud : selected_clouds) {
    filters->setInputCloud(cloud);
    filters->setNegative(ui->check_reverse->isChecked());
    filters->setKeepOrganized(true);
    filters->setUserFilterValue(ui->dspin_user->value());
    switch (ui->cbox_type->currentIndex()) {
      case 0:  // ConditionalRemoval
        cloudview->showInfo("ConditionalRemoval", 1);
        // getCondition();
        emit ConditionalRemoval(getCondition());
        break;
    }
    cloudtree->showProgressBar();
  }
  //    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
  //    if(selectedClouds.empty()) {
  //        console->warning(tr("Please select a pointcloud!"));
  //        return;
  //    }
  //    filtered_clouds.clear();
  //    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
  //    for(size_t i=0;i<selectedClouds.size();i++) {
  //        if(cloud_view->contains(selectedClouds[i]->id))
  //            cloud_tree->setCloudChecked(indexs[i],false);
  //        switch (ui->cbox_filters->currentIndex()){
  //        case 0://PassThrough
  //            emit
  //            passThrough(selectedClouds[i],ui->cbox_fieid_name->currentText().toStdString(),
  //                             ui->dspin_min_limit->value(),ui->dspin_max_limit->value(),ui->check_reverse->isChecked());
  //            cloud_view->showInfo("PassThrough",30,"info");
  //            break;
  //        case 1://VoxelGrid
  //            if(!ui->check_approximate->isChecked())
  //                emit
  //                voxelGrid(selectedClouds[i],ui->dspin_leafx->value(),ui->dspin_leafy->value(),
  //                               ui->dspin_leafz->value(),ui->check_reverse->isChecked());
  //            else
  //                emit
  //                approximateVoxelGrid(selectedClouds[i],ui->dspin_leafx->value(),ui->dspin_leafy->value(),
  //                                          ui->dspin_leafz->value());
  //            cloud_view->showInfo("VoxelGrid",30,"info");
  //            break;
  //        case 2://StatisticalOutlierRemoval
  //            emit
  //            statisticalOutlierRemoval(selectedClouds[i],ui->spin_meank->value(),
  //                                           ui->dspin_stddevmulthresh->value(),ui->check_reverse->isChecked());
  //            cloud_view->showInfo("StatisticalOutlierRemoval",30,"info");
  //            break;
  //        case 3://RadiusOutlierRemoval
  //            emit
  //            radiusOutlierRemoval(selectedClouds[i],ui->dspin_radius->value(),
  //                                      ui->spin_minneiborsinradius->value(),ui->check_reverse->isChecked());
  //            cloud_view->showInfo("RadiusOutlierRemoval",30,"info");
  //            break;
  //        case 4://MovingLeastSquares
  //            emit
  //            movingLeastSquares(selectedClouds[i],ui->check_computeNormals->isChecked(),
  //                                      ui->spin_polynomialOrder->value(),ui->dspin_radius_2->value());
  //            cloud_view->showInfo("RadiusOutlierRemoval",30,"info");
  //            break;
  //        }
  //        if(!ui->check_refresh->isChecked())
  //            console->showProgressBar();
  //    }
}

void Filters::add() {
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

void Filters::apply() {
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

void Filters::reset() {
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

void Filters::filterResult(const pca::Cloud::Ptr &cloud, float time) {
  if (cloud->size() == 0) {
    log(pca::LOG_ERROR, "the pointcloud " + cloud->id() +
                            tr(" filtered failed,take time %1 ms.").arg(time));
    cloudtree->closeProgressBar();
    return;
  }
  filtered_map[cloud->id()] = cloud;
  cloud->setId(cloud->id() + "_filtered");
  cloudview->addPointCloud(cloud);
  cloudview->setPointCloudColor(cloud->id(), 0, 255, 0);
  cloudview->setPointCloudSize(cloud->id(), cloud->pointSize() + 2);
  log(pca::LOG_INFO, "the pointcloud " + cloud->id() +
                         tr(" filtered done,take time %1 ms.").arg(time));
  cloudtree->closeProgressBar();
}

bool Filters::checkValid(bool preview) {
  selected_clouds = cloudtree->getSelectedClouds();
  if (selected_clouds.empty()) {
    log(pca::LOG_WARNING, "please select a pointcloud!");
    return false;
  }
  if (!preview)
    for (auto &cloud : selected_clouds)
      if (filtered_map.find(cloud->id()) == filtered_map.end()) {
        log(pca::LOG_WARNING,
            "please estimation the keypoints or select the correct "
            "pointcloud!");
        return false;
      }
  return true;
}

pca::Condition::Ptr Filters::getCondition() {
  int rowCount = ui->table_condition->rowCount();
  QString condition;
  if (rowCount == 0)
    return nullptr;
  else if (rowCount == 1)
    condition =
        ((QComboBox *)ui->table_condition->cellWidget(0, 0))->currentText();
  else
    condition = ui->table_condition->item(0, 0)->text();
  if (condition == "And") {
    pcl::ConditionAnd<pca::PointXYZRGBN>::Ptr and_cond(
        new pcl::ConditionAnd<pca::PointXYZRGBN>());
    for (int i = 0; i < rowCount; i++) {
      and_cond->addComparison(pcl::FieldComparison<pca::PointXYZRGBN>::ConstPtr(
          new pcl::FieldComparison<pca::PointXYZRGBN>(
              ((QComboBox *)ui->table_condition->cellWidget(i, 1))
                  ->currentText()
                  .toStdString(),
              pcl::ComparisonOps::CompareOp(
                  ((QComboBox *)ui->table_condition->cellWidget(i, 2))
                      ->currentIndex()),
              ui->table_condition->item(i, 3)->text().toFloat())));
    }
    return and_cond;
  } else {
    pcl::ConditionOr<pca::PointXYZRGBN>::Ptr or_cond(
        new pcl::ConditionOr<pca::PointXYZRGBN>());
    for (int i = 0; i < rowCount; i++) {
      or_cond->addComparison(pcl::FieldComparison<pca::PointXYZRGBN>::ConstPtr(
          new pcl::FieldComparison<pca::PointXYZRGBN>(
              ((QComboBox *)ui->table_condition->cellWidget(i, 1))
                  ->currentText()
                  .toStdString(),
              pcl::ComparisonOps::CompareOp(
                  ((QComboBox *)ui->table_condition->cellWidget(i, 2))
                      ->currentIndex()),
              ui->table_condition->item(i, 3)->text().toFloat())));
    }
    return or_cond;
  }
}
