#include "template.h"
#include "ui_template.h"

Template::Template(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Template)
{
    ui->setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);
    this->setWindowFlags(Qt::FramelessWindowHint|Qt::Dialog);
}

Template::~Template()
{
    delete ui;
}

void Template::setCommand()
{
    ui->dspin_speed->hide();
    ui->label->setText("Select Command");
    QTreeWidgetItem* moveJ= new QTreeWidgetItem();
    moveJ->setIcon(0,QIcon(":/icon/resource/icon/format-node-curve.svg"));
    moveJ->setText(0,"MoveJ");
    moveJ->setToolTip(0,"MoveJ");
    QTreeWidgetItem* moveP= new QTreeWidgetItem();
    moveP->setIcon(0,QIcon(":/icon/resource/icon/lines-connector.svg"));
    moveP->setText(0,"MoveP");
    moveP->setToolTip(0,"MoveP");
    QTreeWidgetItem* moveL= new QTreeWidgetItem();
    moveL->setIcon(0,QIcon(":/icon/resource/icon/format-node-line.svg"));
    moveL->setText(0,"MoveL");
    moveL->setToolTip(0,"MoveL");
    QTreeWidgetItem* IO= new QTreeWidgetItem();
    IO->setIcon(0,QIcon(":/icon/resource/icon/network-connect.svg"));
    IO->setText(0,"IOWrite");
    IO->setToolTip(0,"IOWrite");
    QTreeWidgetItem* Capture= new QTreeWidgetItem();
    Capture->setIcon(0,QIcon(":/icon/resource/icon/device.svg"));
    Capture->setText(0,"Capture");
    Capture->setToolTip(0,"Capture");
    ui->tree_template->addTopLevelItem(moveJ);
    ui->tree_template->addTopLevelItem(moveP);
    ui->tree_template->addTopLevelItem(moveL);
    ui->tree_template->addTopLevelItem(IO);
    ui->tree_template->addTopLevelItem(Capture);
}

void Template::setAllPath(int size)
{
    ui->dspin_speed->show();
    ui->label->setText("Select path point");
    QIcon item_icon(":/icon/resource/icon/format-convert-to-path.svg");
    for(int i=0;i<size;i++){
        QTreeWidgetItem* item= new QTreeWidgetItem();
        item->setText(0,"path_"+QString::number(i));
        item->setIcon(0,item_icon);
        ui->tree_template->addTopLevelItem(item);
    }
}

void Template::setTemplate(const std::vector<Cloud::Ptr> &all_clouds)
{
    ui->dspin_speed->hide();
    ui->label->setText("Select template cloud");
    QIcon item_icon(":/icon/resource/icon/view-calendar.svg");
    for(auto &i:all_clouds){
        QTreeWidgetItem* item= new QTreeWidgetItem();
        item->setText(0,i->id.c_str());
        item->setIcon(0,item_icon);
        ui->tree_template->addTopLevelItem(item);
    }
}

void Template::setTemplateWithDevice(const std::vector<Cloud::Ptr> &all_clouds)
{
    ui->dspin_speed->hide();
    ui->label->setText("Select target cloud");
    QIcon device_icon(":/icon/resource/icon/device.svg");
    QIcon child_icon(":/icon/resource/icon/view-calendar.svg");
    QTreeWidgetItem* device= new QTreeWidgetItem();
    device->setText(0,"Zhisensor");
    device->setIcon(0,device_icon);
    ui->tree_template->addTopLevelItem(device);
    for(auto &i:all_clouds){
        QTreeWidgetItem* item= new QTreeWidgetItem();
        item->setText(0,i->id.c_str());
        item->setIcon(0,child_icon);
        ui->tree_template->addTopLevelItem(item);
    }
}

void Template::setExtendedSelection(bool enable)
{
    if(enable)
        ui->tree_template->setSelectionMode(QAbstractItemView::ExtendedSelection);
    else
        ui->tree_template->setSelectionMode(QAbstractItemView::SingleSelection);
}

void Template::accept()
{
    std::vector<int> index;
    auto items=ui->tree_template->selectedItems();
    if(items.size()>0) {
        for(auto &i:items) {
            int k=ui->tree_template->indexOfTopLevelItem(i);
            index.push_back(k);
        }
        emit getIndex(index.front());
        emit getTemplate(index);
        emit getPath(index.front(),ui->dspin_speed->value());
    }
    return QDialog::accept();
}
