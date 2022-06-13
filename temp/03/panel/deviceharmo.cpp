#include "deviceharmo.h"
#include "ui_deviceharmo.h"

DeviceHarmo::DeviceHarmo(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::DeviceHarmo),is_connected(true),is_poweron(false),is_enbled(false),
    is_run(false),is_readstate(false),is_drag(false),capture_cloud(false),current_io_num(0),current_move_num(0)
{
    ui->setupUi(this);
    ui->table_robot_state->resizeColumnsToContents();
    connect(ui->btn_connect,&QPushButton::clicked,this,&DeviceHarmo::connectRobot);
    connect(ui->btn_power,&QPushButton::clicked,this,&DeviceHarmo::powerOn);
    connect(ui->btn_enable,&QPushButton::clicked,this,&DeviceHarmo::enable);
    connect(ui->btn_reset,&QPushButton::clicked,this,&DeviceHarmo::reset);
    connect(ui->btn_io,&QPushButton::clicked,this,&DeviceHarmo::ioWrite);
    connect(ui->btn_movej,&QPushButton::clicked,this,&DeviceHarmo::moveJ);
    connect(ui->btn_movep,&QPushButton::clicked,this,&DeviceHarmo::moveP);
    connect(ui->btn_movel,&QPushButton::clicked,this,&DeviceHarmo::moveL);
    connect(ui->btn_drag,&QPushButton::clicked,this,&DeviceHarmo::drag);
    connect(ui->btn_stop,&QPushButton::clicked,this,&DeviceHarmo::stop);
    connect(ui->btn_soft_stop,&QPushButton::clicked,this,&DeviceHarmo::softStop);
    connect(ui->btn_run,&QPushButton::clicked,this,&DeviceHarmo::run);
    connect(ui->btn_read_state,&QPushButton::clicked,this,&DeviceHarmo::readState);
    Harmo_Read =new DeviceHarmoRead();
    Harmo_Read->moveToThread(&thread);
    qRegisterMetaType<st_JointAngle>("st_JointAngle &");
    qRegisterMetaType<st_JointAngle>("st_JointAngle");
    qRegisterMetaType<st_SpaceCoordinate>("st_SpaceCoordinate &");
    qRegisterMetaType<st_SpaceCoordinate>("st_SpaceCoordinate");
    connect(&thread,&QThread::finished,Harmo_Read,&QObject::deleteLater);
    connect(this,&DeviceHarmo::startReadState,Harmo_Read,&DeviceHarmoRead::updateState);
    connect(this,&DeviceHarmo::login,Harmo_Read,&DeviceHarmoRead::login);
    connect(this,&DeviceHarmo::logout,Harmo_Read,&DeviceHarmoRead::logout);
    connect(Harmo_Read,&DeviceHarmoRead::state,this,&DeviceHarmo::updateState);
    connect(Harmo_Read,&DeviceHarmoRead::loginResult,this,&DeviceHarmo::loginResult);
    connect(Harmo_Read,&DeviceHarmoRead::logoutResult,this,&DeviceHarmo::logoutResult);
    thread.start();
}

DeviceHarmo::~DeviceHarmo()
{
    Harmo_Read->is_stopped=true;
    thread.quit();
    thread.wait();
    delete ui;
}

void DeviceHarmo::init(Console *&co,PathTable *ptb,ProcessTree* pt)
{
    console=co;path_table=ptb;process_tree=pt;
    connect(process_tree,&ProcessTree::moveJ,this,&DeviceHarmo::moveJ);
    connect(process_tree,&ProcessTree::moveP,this,&DeviceHarmo::moveP);
    connect(process_tree,&ProcessTree::moveL,this,&DeviceHarmo::moveL);
    connect(process_tree,&ProcessTree::ioWrite,this,&DeviceHarmo::ioWrite);
    connect(process_tree,&ProcessTree::startMove,this,&DeviceHarmo::startMove);
}

void DeviceHarmo::connectRobot()
{
    if(!is_connected) {
        if(Harmo_Control.robotServiceLogin()){
            console->info(tr("Connect the robot sucessfully!"));
            ui->btn_connect->setText(tr("Discon"));
            ui->btn_connect->setIcon(QIcon(":/icon/resource/icon/disconnect.svg"));
            Harmo_Read->is_stopped=false;
            emit login();
        } else {
            console->error(tr("Connect the robot failed."));
            return ;
        }
    } else {
        if(Harmo_Control.robotServiceLogout()){
            console->info(tr("Disconnect the device sucessfully!"));
            ui->btn_connect->setText(tr("Connect"));
            ui->btn_connect->setIcon(QIcon(":/icon/resource/icon/connect.svg"));
            Harmo_Read->is_stopped=true;
            emit logout();
        } else {
            console->error(tr("Disconnect the robot failed."));
            return ;
        }
    }
}

void DeviceHarmo::powerOn()
{
    if(is_connected) {
        if(!is_poweron){
            if(Harmo_Control.robotServicePowerOnPack(1)){
                console->info(tr("The robot power on sucessfully!"));
                ui->btn_power->setText(tr("PowerOff"));
                ui->btn_power->setIcon(QIcon(":/icon/resource/icon/power_off.svg"));
                is_poweron=true;
            } else {
                console->error(tr("The robot power on failed."));
                return ;
            }
        } else {
            if(Harmo_Control.robotServicePowerOnPack(0)){
                console->info(tr("The robot power off sucessfully!"));
                ui->btn_power->setText(tr("PowerOn"));
                ui->btn_power->setIcon(QIcon(":/icon/resource/icon/power_on.svg"));
                is_poweron=false;
            } else {
                console->error(tr("The robot power off failed."));
                return ;
            }
        }
    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}

void DeviceHarmo::enable()
{
    if(is_connected) {
        if(!is_enbled){
            if(Harmo_Control.robotServiceEnableCommand(1)){
                console->info(tr("The robot enable sucessfully!"));
                ui->btn_enable->setText(tr("Disable"));
                ui->btn_enable->setIcon(QIcon(":/icon/resource/icon/disable.svg"));
                is_enbled=true;
            } else {
                console->error(tr("The robot enable failed."));
                return ;
            }
        } else {
            if(Harmo_Control.robotServiceEnableCommand(0)){
                console->info(tr("The robot disable sucessfully!"));
                ui->btn_enable->setText(tr("Enable"));
                ui->btn_enable->setIcon(QIcon(":/icon/resource/icon/enable.svg"));
                is_enbled=false;
            } else {
                console->error(tr("The robot disable failed."));
                return ;
            }
        }
    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}

void DeviceHarmo::reset()
{
    if(is_connected) {
        if(Harmo_Control.robotServiceResetCommand())
            console->info(tr("The robot reset command sucessfully!"));
        else
            console->error(tr("The robot reset command failed."));
    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}

void DeviceHarmo::ioWrite()
{
    if(is_connected) {
        MoveData *dialog=new MoveData(this);
        dialog->setIO();
        connect(dialog,&MoveData::ioData,[=](const IOWrite&io){
            if(io.type==Empty){
                console->warning(tr("IOCommand format is wrong ,please retry"));
                return;
            }
            qDebug()<<io.num<<io.io_type<<io.count<<io.value;
            if(Harmo_Control.robotServiceIoCommandWrite(io.num,io.io_type,io.count,io.value))
                console->info(tr("The robot io write sucessfully!"));
            else
                console->error(tr("The robot io write failed."));
        });
        connect(dialog,&MoveData::addIoData,[=](const IOWrite&io){
            if(io.type==Empty){
                console->warning(tr("IOCommand format is wrong ,please retry"));
                return;
            }
            qDebug()<<io.num<<io.io_type<<io.count<<io.value;
            path_table->addIO(io);
        });
        dialog->show();

    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}

void DeviceHarmo::moveJ()
{
    if(is_connected) {
        MoveData *dialog=new MoveData(this);
        dialog->setMoveJ();
        connect(dialog,&MoveData::moveData,[=](const Position&pos){
            if(pos.type==Empty){
                console->warning(tr("MoveJ position format is wrong ,please retry"));
                return;
            }
            qDebug()<<pos.x<<pos.y<<pos.z<<pos.rx<<pos.ry<<pos.rz<<pos.v;
            if(Harmo_Control.robotServiceMoveJData(pos.x,pos.y,pos.z,pos.rx,pos.ry,pos.rz,pos.v,0))
                console->info(tr("The robot MoveJ command sucessfully!"));
            else
                console->error(tr("The robot MoveJ command failed."));
        });
        connect(dialog,&MoveData::addMoveData,[=](const Position&pos){
            if(pos.type==Empty){
                console->warning(tr("MoveJ position format is wrong ,please retry"));
                return;
            }
            qDebug()<<pos.x<<pos.y<<pos.z<<pos.rx<<pos.ry<<pos.rz<<pos.v;
            path_table->addMove(pos);
        });
        dialog->show();

    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}

void DeviceHarmo::moveP()
{
    if(is_connected) {
        MoveData *dialog=new MoveData(this);
        dialog->setMoveP();
        connect(dialog,&MoveData::moveData,[=](const Position&pos){
            if(pos.type==Empty){
                console->warning(tr("MoveP position format is wrong ,please retry"));
                return;
            }
            qDebug()<<pos.x<<pos.y<<pos.z<<pos.rx<<pos.ry<<pos.rz<<pos.v;
            if(Harmo_Control.robotServiceMovePData(pos.x,pos.y,pos.z,pos.rx,pos.ry,pos.rz,pos.v))
                console->info(tr("The robot MoveP command sucessfully!"));
            else
                console->error(tr("The robot MoveP command failed."));
        });
        connect(dialog,&MoveData::addMoveData,[=](const Position&pos){
            if(pos.type==Empty){
                console->warning(tr("MoveP position format is wrong ,please retry"));
                return;
            }
            qDebug()<<pos.x<<pos.y<<pos.z<<pos.rx<<pos.ry<<pos.rz<<pos.v;
            path_table->addMove(pos);
        });
        dialog->show();

    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }

}

void DeviceHarmo::moveL()
{
    if(is_connected) {
        MoveData *dialog=new MoveData(this);
        dialog->setMoveL();
        connect(dialog,&MoveData::moveData,[=](const Position&pos){
            if(pos.type==Empty){
                console->warning(tr("MoveL position format is wrong ,please retry"));
                return;
            }
            qDebug()<<pos.x<<pos.y<<pos.z<<pos.rx<<pos.ry<<pos.rz<<pos.v;
            if(Harmo_Control.robotServiceMoveLDataPack(pos.x,pos.y,pos.z,pos.rx,pos.ry,pos.rz,pos.v))
                console->info(tr("The robot MoveL command sucessfully!"));
            else
                console->error(tr("The robot MoveL command failed."));
        });
        connect(dialog,&MoveData::addMoveData,[=](const Position&pos){
            if(pos.type==Empty){
                console->warning(tr("MoveL position format is wrong ,please retry"));
                return;
            }
            qDebug()<<pos.x<<pos.y<<pos.z<<pos.rx<<pos.ry<<pos.rz<<pos.v;
            path_table->addMove(pos);
        });
        dialog->show();

    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}

void DeviceHarmo::drag()
{
    if(is_connected) {
        if(!is_drag){
            if(Harmo_Control.robotServiceDragCommand(1)){
                console->info(tr("The robot drag command sucessfully!"));
                ui->btn_drag->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
                is_drag=true;
            } else {
                console->error(tr("The robot run command failed."));
                return ;
            }
        } else {
            if(Harmo_Control.robotServiceDragCommand(0)){
                console->info(tr("The robot drag run sucessfully!"));
                ui->btn_drag->setIcon(QIcon(":/icon/resource/icon/robot.svg"));
                is_drag=false;
            } else {
                console->error(tr("The robot stop run failed."));
                return ;
            }
        }
    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}

void DeviceHarmo::stop()
{
    if(is_connected) {
        if(Harmo_Control.robotServiceStopCommand()){
            console->info(tr("The robot stop command sucessfully!"));
        } else {
            console->error(tr("The robot stop command failed."));
            return ;
        }
    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}

void DeviceHarmo::softStop()
{
    if(is_connected) {
        if(Harmo_Control.robotServiceSoftStop()){
            console->info(tr("The robot soft stop command sucessfully!"));
        } else {
            console->error(tr("The robot soft stop command failed."));
            return ;
        }
    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}

void DeviceHarmo::run()
{
    if(is_connected) {
        if(!is_run){
            if(Harmo_Control.robotServiceRunCommand(1)){
                console->info(tr("The robot run command sucessfully!"));
                ui->btn_run->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
                is_run=true;
            } else {
                console->error(tr("The robot run command failed."));
                return ;
            }
        } else {
            if(Harmo_Control.robotServiceRunCommand(0)){
                console->info(tr("The robot stop run sucessfully!"));
                ui->btn_run->setIcon(QIcon(":/icon/resource/icon/start.svg"));
                is_run=false;
            } else {
                console->error(tr("The robot stop run failed."));
                return ;
            }
        }
    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}

void DeviceHarmo::readState()
{
    if(!is_readstate) {
        Harmo_Read->is_stopped=false;
        ui->btn_read_state->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
        is_readstate=true;
        emit startReadState();
    }
    else {
        Harmo_Read->is_stopped=true;
        ui->btn_read_state->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        is_readstate=false;
    }
}

void DeviceHarmo::loginResult(bool sucess)
{
    if(sucess){
        console->info(tr("The robot read state connect sucessfully!"));
        is_connected=true;
    } else {
        console->error(tr("The robot read state connect failed."));
        return ;
    }
}

void DeviceHarmo::logoutResult(bool sucess)
{
    if(sucess){
        console->info(tr("The robot read state disconnect sucessfully!"));
        is_connected=false;
    } else {
        console->error(tr("The robot read state disconnect failed."));
        return ;
    }
}

void DeviceHarmo::updateState(bool sucess, const st_SpaceCoordinate &space, const st_JointAngle &joint)
{
    if(sucess){
        ui->table_robot_state->setItem(0,1,new QTableWidgetItem(QString::number(space.fX)));
        ui->table_robot_state->setItem(1,1,new QTableWidgetItem(QString::number(space.fY)));
        ui->table_robot_state->setItem(2,1,new QTableWidgetItem(QString::number(space.fZ)));
        ui->table_robot_state->setItem(3,1,new QTableWidgetItem(QString::number(space.fRX)));
        ui->table_robot_state->setItem(4,1,new QTableWidgetItem(QString::number(space.fRY)));
        ui->table_robot_state->setItem(5,1,new QTableWidgetItem(QString::number(space.fRZ)));
        ui->table_robot_state->setItem(0,3,new QTableWidgetItem(QString::number(joint.fOneAxis)));
        ui->table_robot_state->setItem(1,3,new QTableWidgetItem(QString::number(joint.fTwoAxis)));
        ui->table_robot_state->setItem(2,3,new QTableWidgetItem(QString::number(joint.fThreeAxis)));
        ui->table_robot_state->setItem(3,3,new QTableWidgetItem(QString::number(joint.fFourAxis)));
        ui->table_robot_state->setItem(4,3,new QTableWidgetItem(QString::number(joint.fFiveAxis)));
        ui->table_robot_state->setItem(5,3,new QTableWidgetItem(QString::number(joint.fSixAxis)));
        emit robotState(space,joint);
    }else {
        console->error(tr("The robot read state start failed."));
        Harmo_Read->is_stopped=true;
        ui->btn_read_state->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        is_readstate=false;
        return ;
    }
}

void DeviceHarmo::startIOWrite(const st_SpaceCoordinate &space, const st_JointAngle &joint)
{
    if(current_io_num<current_io.size())
        switch (current_io[current_io_num].pos.type) {
        case MoveJ:
            if(((joint.fOneAxis-0.1)<current_io[current_io_num].pos.x)&&(current_io[current_io_num].pos.x<(joint.fOneAxis+0.1))&&
                    ((joint.fTwoAxis-0.1)<current_io[current_io_num].pos.y)&&(current_io[current_io_num].pos.y<(joint.fTwoAxis+0.1))&&
                    ((joint.fThreeAxis-0.1)<current_io[current_io_num].pos.z)&&(current_io[current_io_num].pos.z<(joint.fThreeAxis+0.1))&&
                    ((joint.fFourAxis-0.1)<current_io[current_io_num].pos.rx)&&(current_io[current_io_num].pos.rx<(joint.fFourAxis+0.1))&&
                    ((joint.fFiveAxis-0.1)<current_io[current_io_num].pos.ry)&&(current_io[current_io_num].pos.ry<(joint.fFiveAxis+0.1))&&
                    ((joint.fSixAxis-0.1)<current_io[current_io_num].pos.rz)&&(current_io[current_io_num].pos.rz<(joint.fSixAxis+0.1)))
                if(Harmo_Control.robotServiceIoCommandWrite(current_io[current_io_num].num,current_io[current_io_num].io_type,current_io[current_io_num].count,current_io[current_io_num].value)) {
                    console->info(tr("The robot io write sucessfully!"));
                    current_io_num--;
                }
                else
                    console->error(tr("The robot io write failed."));
            break;
        default:
            if(((space.fX-0.1)<current_io[current_io_num].pos.x)&&(current_io[current_io_num].pos.x<(space.fX+0.1))&&
                    ((space.fY-0.1)<current_io[current_io_num].pos.y)&&(current_io[current_io_num].pos.y<(space.fY+0.1))&&
                    ((space.fZ-0.1)<current_io[current_io_num].pos.z)&&(current_io[current_io_num].pos.z<(space.fZ+0.1))&&
                    ((space.fRX-0.1)<current_io[current_io_num].pos.rx)&&(current_io[current_io_num].pos.rx<(space.fRX+0.1))&&
                    ((space.fRY-0.1)<current_io[current_io_num].pos.ry)&&(current_io[current_io_num].pos.ry<(space.fRY+0.1))&&
                    ((space.fRZ-0.1)<current_io[current_io_num].pos.rz)&&(current_io[current_io_num].pos.rz<(space.fRZ+0.1)))
                if(Harmo_Control.robotServiceIoCommandWrite(current_io[current_io_num].num,current_io[current_io_num].io_type,current_io[current_io_num].count,current_io[current_io_num].value)) {
                    console->info(tr("The robot io write sucessfully!"));
                    current_io_num--;
                }
                else
                    console->error(tr("The robot io write failed."));
            break;
        }

    switch (current_move.back().type){
    case MoveJ:
        if(((joint.fOneAxis-0.1)<current_move.back().x)&&(current_move.back().x<(joint.fOneAxis+0.1))&&
                ((joint.fTwoAxis-0.1)<current_move.back().y)&&(current_move.back().y<(joint.fTwoAxis+0.1))&&
                ((joint.fThreeAxis-0.1)<current_move.back().z)&&(current_move.back().z<(joint.fThreeAxis+0.1))&&
                ((joint.fFourAxis-0.1)<current_move.back().rx)&&(current_move.back().rx<(joint.fFourAxis+0.1))&&
                ((joint.fFiveAxis-0.1)<current_move.back().ry)&&(current_move.back().ry<(joint.fFiveAxis+0.1))&&
                ((joint.fSixAxis-0.1)<current_move.back().rz)&&(current_move.back().rz<(joint.fSixAxis+0.1))){
            if(capture_cloud)
                emit process_tree->lastMoveDoneBeforeCapture();
            else
                emit process_tree->lastMoveDoneAfterCapture();
            disconnect(this,&DeviceHarmo::robotState,this,&DeviceHarmo::startIOWrite);
        }
        break;
    default:
        if(((space.fX-0.1)<current_move.back().x)&&(current_move.back().x<(space.fX+0.1))&&
                ((space.fY-0.1)<current_move.back().y)&&(current_move.back().y<(space.fY+0.1))&&
                ((space.fZ-0.1)<current_move.back().z)&&(current_move.back().z<(space.fZ+0.1))&&
                ((space.fRX-0.1)<current_move.back().rx)&&(current_move.back().rx<(space.fRX+0.1))&&
                ((space.fRY-0.1)<current_move.back().ry)&&(current_move.back().ry<(space.fRY+0.1))&&
                ((space.fRZ-0.1)<current_move.back().rz)&&(current_move.back().rz<(space.fRZ+0.1))){
            if(capture_cloud)
                emit process_tree->lastMoveDoneBeforeCapture();
            else
                emit process_tree->lastMoveDoneAfterCapture();
            disconnect(this,&DeviceHarmo::robotState,this,&DeviceHarmo::startIOWrite);
        }
        break;
    }
}

void DeviceHarmo::startMove(const std::vector<Position> & pos, const std::vector<IOWrite> & io,bool capture_cloud_)
{
    if(is_connected) {
        for(size_t i=0;i<pos.size();i++)
        {
            if(i==0)
            {
                if(Harmo_Control.robotServiceRunCommand(1))
                    console->info(tr("The robot run command sucessfully!"));
                else {
                    console->error(tr("The robot run command failed."));
                    return;
                }
            }
            switch (pos[i].type) {
            case MoveJ:
                if(Harmo_Control.robotServiceMoveJData(pos[i].x,pos[i].y,pos[i].z,pos[i].rx,pos[i].ry,pos[i].rz,pos[i].v,0))
                    console->info(tr("The robot MoveJ command %1 %2 %3 %4 %5 %6 （Speed: %7)").arg(pos[i].x).arg(pos[i].y)
                                  .arg(pos[i].z).arg(pos[i].rx).arg(pos[i].rz).arg(pos[i].v)+" sucessfully!");
                else
                    console->info(tr("The robot MoveJ command %1 %2 %3 %4 %5 %6 （Speed: %7)").arg(pos[i].x).arg(pos[i].y)
                                  .arg(pos[i].z).arg(pos[i].rx).arg(pos[i].rz).arg(pos[i].v)+" failed!");
                break;
            case MoveP:
                if(Harmo_Control.robotServiceMovePData(pos[i].x,pos[i].y,pos[i].z,pos[i].rx,pos[i].ry,pos[i].rz,pos[i].v))
                    console->info(tr("The robot MoveP command %1 %2 %3 %4 %5 %6 （Speed: %7)").arg(pos[i].x).arg(pos[i].y)
                                  .arg(pos[i].z).arg(pos[i].rx).arg(pos[i].rz).arg(pos[i].v)+" sucessfully!");
                else
                    console->info(tr("The robot MoveP command %1 %2 %3 %4 %5 %6 （Speed: %7)").arg(pos[i].x).arg(pos[i].y)
                                  .arg(pos[i].z).arg(pos[i].rx).arg(pos[i].rz).arg(pos[i].v)+" failed!");
                break;
            case MoveL:
                if(Harmo_Control.robotServiceMoveLDataPack(pos[i].x,pos[i].y,pos[i].z,pos[i].rx,pos[i].ry,pos[i].rz,pos[i].v))
                    console->info(tr("The robot MoveL command %1 %2 %3 %4 %5 %6 （Speed: %7)").arg(pos[i].x).arg(pos[i].y)
                                  .arg(pos[i].z).arg(pos[i].rx).arg(pos[i].rz).arg(pos[i].v)+" sucessfully!");
                else
                    console->info(tr("The robot MoveL command %1 %2 %3 %4 %5 %6 （Speed: %7)").arg(pos[i].x).arg(pos[i].y)
                                  .arg(pos[i].z).arg(pos[i].rx).arg(pos[i].rz).arg(pos[i].v)+" failed!");
                break;
            }
        }
        capture_cloud=capture_cloud_;
        current_io=io;current_io_num=current_io.size();
        current_move=pos;current_move_num=current_move.size();
        connect(this,&DeviceHarmo::robotState,this,&DeviceHarmo::startIOWrite);
    } else {
        console->warning(tr("Please connect a robot."));
        return;
    }
}
