#include "colors.h"
#include "ui_colors.h"

Colors::Colors(QWidget *parent) :QDockWidget(parent),
    ui(new Ui::Colors),field_name(""),red(-1),green(-1),blue(-1)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&Colors::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Colors::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Colors::reset);
    ui->gridLayout->setSpacing(0);
    for (int row=0;row<5;row++){
        for(int column=0;column<10;column++){
            QPushButton *btn = new QPushButton();
            btn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            btn->setFixedHeight(20);
            if(row==4 && column==9 ){
                btn->setText("+");
                btn->setStyleSheet(tr("QPushButton{border:none;border-radius:4px;background-color:transparent;}"
                                      "QPushButton:pressed{background-color:lightgray;}"));
                connect(btn,&QPushButton::clicked,[=]{
                    QColor color=QColorDialog::getColor(Qt::white,this, tr("select color"));
                    if(ui->cbox_type->currentIndex()==0)
                        emit cloudrgb(color.red(),color.green(),color.blue());
                    else if(ui->cbox_type->currentIndex()==1)
                        emit backgroundrgb(color.red(),color.green(),color.blue());
                });
            }
            else {
                btn->setStyleSheet(tr("QPushButton{border:none;border-radius:4px;background-color:rgb(%1, %2, %3);}"
                                      "QPushButton:pressed{background-color:lightgray;}").
                                   arg((colors[row][column]).red()).arg((colors[row][column]).green()).
                                   arg((colors[row][column]).blue()));
                connect(btn,&QPushButton::clicked,[=]{
                    QColor color=colors[row][column];
                    if(ui->cbox_type->currentIndex()==0)
                        emit cloudrgb(color.red(),color.green(),color.blue());
                    else if(ui->cbox_type->currentIndex()==1)
                        emit backgroundrgb(color.red(),color.green(),color.blue());
                });
            }
            ui->gridLayout->addWidget(btn,row,column);
        }
    }
    connect(ui->btn_x,&QPushButton::clicked,[=]{emit fieldname("x");});
    connect(ui->btn_y,&QPushButton::clicked,[=]{emit fieldname("y");});
    connect(ui->btn_z,&QPushButton::clicked,[=]{emit fieldname("z");});
}

Colors::~Colors()
{
    delete ui;
}

void Colors::init(Console* &co,CloudView* &cv,CloudTree* &ct,ProcessTree* pt)
{
    console=co;cloud_view=cv;cloud_tree=ct;process_tree=pt;
    connect(this,&Colors::cloudrgb,[=](int r,int g,int b){
        red=r;green=g;blue=b;field_name="";
        std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
        if(selectedClouds.empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        for (auto&i:selectedClouds)
            cloud_view->setCloudColor(i,i->id,r,g,b);
    });
    connect(this,&Colors::backgroundrgb,[=](int r,int g,int b){
        cloud_view->setBackgroundColor(r,g,b);
    });
    connect(this,&Colors::fieldname,[=](const std::string &name){
        field_name=name;red=green=blue=-1;
        std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
        if(selectedClouds.empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        for (auto&i:selectedClouds)
            cloud_view->setCloudColor(i,i->id,name);
    });

}

void Colors::add()
{
    if(process_tree->enable()){
        if(red==-1&&field_name=="") {
            console->warning(tr("Please select a color!"));
            return;
        }
        Process::Ptr colors(new Process);
        colors->id="Colors";
        colors->type=process_color;
        colors->icon=QIcon(":/icon/resource/icon/color-management.svg");
        colors->value.resize(4);
        if(field_name=="") colors->value[0]=-1;
        if(field_name=="x") colors->value[0]=0;
        if(field_name=="y") colors->value[0]=1;
        if(field_name=="z") colors->value[0]=2;
        colors->value[1]=red;
        colors->value[2]=green;
        colors->value[3]=blue;
        process_tree->insertProcess(colors);
        return;
    }
}

void Colors::apply()
{
    if(red==-1&&field_name=="") {
        console->warning(tr("Please select a color!"));
        return;
    }
    if(ui->cbox_type->currentIndex()==0) {
        std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
        if(selectedClouds.empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        for (auto&i:selectedClouds) {
            if(red!=-1){
                Common::setColor(i,red,green,blue);
                console->info(tr("The pointcloud ")+i->id.c_str()+tr(" color has changed to rgb(%1,%2,%3).").arg(red).arg(green).arg(blue));
                cloud_view->updateCloud(i,i->id);
            } else {
                Common::setColor(i,field_name);
                console->info(tr("The pointcloud ")+i->id.c_str()+tr(" color has changed along the ")+field_name.c_str());
                cloud_view->updateCloud(i,i->id);
            }
        }
    }
}

void Colors::reset()
{
    if(red==-1&&field_name=="") return;
    if(ui->cbox_type->currentIndex()==0) {
        std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
        for (auto&i:selectedClouds)
            cloud_view->resetCloudColor(i,i->id);
    } else
        cloud_view->resetBackgroundColor();
    red=green=blue=-1;field_name="";
}

void Colors::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QDockWidget::closeEvent(event);
}
