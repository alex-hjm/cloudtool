#include "movedata.h"
#include "ui_movedata.h"

MoveData::MoveData(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MoveData),current_type(-1)
{
    ui->setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);
    this->setWindowFlags(Qt::FramelessWindowHint|Qt::Dialog);
    connect(ui->btn_add,&QPushButton::clicked,this,&MoveData::add);
    connect(ui->btn_move,&QPushButton::clicked,this,&MoveData::move);
    connect(ui->btn_cancel,&QPushButton::clicked,this,&MoveData::close);
}

MoveData::~MoveData()
{
    delete ui;
}

void MoveData::setMoveJ()
{
    current_type=0;
    ui->label->setText("MoveJ");
    ui->lineEdit_pos->setPlaceholderText("joint1 joint2 joint3 joint4 joint5 joint6");
    ui->widgetIO->hide();
    this->setFixedSize(450,85);
}

void MoveData::setMoveP()
{
    current_type=1;
    ui->label->setText("MoveP");
    ui->lineEdit_pos->setPlaceholderText("X Y Z RX RY RZ");
    ui->widgetIO->hide();
    this->setFixedSize(450,85);
}

void MoveData::setMoveL()
{
    current_type=2;
    ui->label->setText("MoveL");
    ui->lineEdit_pos->setPlaceholderText("X Y Z RX RY RZ");
    ui->widgetIO->hide();
    this->setFixedSize(450,85);
}

void MoveData::setIO()
{
    current_type=3;
    ui->label->setText("IO Write");
    ui->btn_move->setText("Set");
    ui->lineEdit_pos->hide();ui->dspin_speed->hide();
    this->setFixedSize(450,85);
}

void MoveData::add()
{
    Position pos;
    QStringList textlist = ui->lineEdit_pos->text().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
    if (textlist.size() != 6) {
        emit addMoveData(pos);
    } else {
        pos.x=textlist[0].toFloat();
        pos.y=textlist[1].toFloat();
        pos.z=textlist[2].toFloat();
        pos.rx=textlist[3].toFloat();
        pos.ry=textlist[4].toFloat();
        pos.rz=textlist[5].toFloat();
        pos.v=ui->dspin_speed->value();
        if(current_type==0){
            if((0<pos.x&&pos.x<360)&&(0<pos.y&&pos.y<360)&&(0<pos.z&&pos.z<360)
                    &&(0<pos.rx&&pos.rx<360)&&(0<pos.ry&&pos.ry<360)&&(0<pos.rz&&pos.rz<360)&&(0<pos.v)){
                pos.type=command_type(current_type);
                emit addMoveData(pos);
            } else emit moveData(pos);
        } else if(current_type==1 || current_type==2){
            if((-180<pos.rx&&pos.rx<180)&&(-180<pos.ry&&pos.ry<180)&&(-180<pos.rz&&pos.rz<180)&&(pos.v>0)){
                pos.type=command_type(current_type);
                emit addMoveData(pos);
            } else emit addMoveData(pos);
        }
    }

    IOWrite io;
    io.num=ui->spin_io_num->value();
    io.io_type=ui->cbox_io_type->currentIndex();
    io.count=ui->spin_io_count->value();
    io.value=ui->dspin_io_value->value();
    if(current_type==3)  {
        io.type=command_type(current_type);
        emit addIoData(io);
    }
}

void MoveData::move()
{
    Position pos;
    QStringList textlist = ui->lineEdit_pos->text().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
    if (textlist.size() != 6) {
        emit moveData(pos);
    } else {
        pos.x=textlist[0].toFloat();
        pos.y=textlist[1].toFloat();
        pos.z=textlist[2].toFloat();
        pos.rx=textlist[3].toFloat();
        pos.ry=textlist[4].toFloat();
        pos.rz=textlist[5].toFloat();
        pos.v=ui->dspin_speed->value();
        if(current_type==0){
            if((-180<pos.x&&pos.x<180)&&(-180<pos.y&&pos.y<180)&&(-180<pos.z&&pos.z<180)
                    &&(-180<pos.rx&&pos.rx<180)&&(-180<pos.ry&&pos.ry<180)&&(-180<pos.rz&&pos.rz<180)&&(0<pos.v)){
                pos.type=command_type(current_type);
                emit moveData(pos);
            } else emit moveData(pos);
        } else if(current_type==1 || current_type==2){
            if((-180<pos.rx&&pos.rx<180)&&(-180<pos.ry&&pos.ry<180)&&(-180<pos.rz&&pos.rz<180)&&(pos.v>0)){
                pos.type=command_type(current_type);
                emit moveData(pos);
            } else emit moveData(pos);
        }
    }

    IOWrite io;
    io.num=ui->spin_io_num->value();
    io.io_type=ui->cbox_io_type->currentIndex()+1;
    io.count=ui->spin_io_count->value();
    io.value=ui->dspin_io_value->value();
    if(current_type==3)  {
        io.type=command_type(current_type);
        emit ioData(io);
    }
}
