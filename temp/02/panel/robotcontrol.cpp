#include "robotcontrol.h"
#include "ui_robotcontrol.h"

RobotControl::RobotControl(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RobotControl)
{
    ui->setupUi(this);
}

RobotControl::~RobotControl()
{
    delete ui;
}
