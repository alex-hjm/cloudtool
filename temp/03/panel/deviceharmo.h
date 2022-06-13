#ifndef DEVICEHARMO_H
#define DEVICEHARMO_H

#include <QDockWidget>
#include <QTimer>
#define NOMINMAX
#include <TcpClientControlOperate.h>
#include <TcpClientStateOperate.h>
#include "common/cloudtree.h"
#include "common/processtree.h"
#include "panel/movedata.h"

class DeviceHarmoRead : public QObject
{
    Q_OBJECT
public:
    explicit DeviceHarmoRead(QWidget *parent = nullptr):QObject(parent),is_stopped(false){}

    void login()
    {
        if(!Harmo_State.robotServiceLogin())
            emit loginResult(false);
        else
            emit loginResult(true);
    }

    void logout()
    {
        if(!Harmo_State.robotServiceLogout())
            emit logoutResult(false);
        else
            emit logoutResult(true);
    }

    void updateState()
    {
        st_JointAngle joint;
        st_SpaceCoordinate space;
        while(!is_stopped){
            if(!Harmo_State.robotServiceGetJointAngle(joint)) {
                emit state(false,space,joint);
            }
            if(!Harmo_State.robotServiceGetSpacePosition(space)) {
                emit state(false,space,joint);
            }
            emit state(true,space,joint);
        }
    }


signals:
    void loginResult(bool sucess);
    void logoutResult(bool sucess);
    void state(bool sucess,const st_SpaceCoordinate&space,const st_JointAngle&joint);

public:
    bool is_stopped;
    CTcpClientStateOperate Harmo_State;
};


namespace Ui {
class DeviceHarmo;
}

class DeviceHarmo : public QDockWidget
{
    Q_OBJECT

public:
    explicit DeviceHarmo(QWidget *parent = nullptr);
    ~DeviceHarmo();
    void init(Console* &co,PathTable *ptb,ProcessTree* pt);
    void connectRobot();
    void powerOn();
    void enable();
    void reset();
    void ioWrite();
    void moveJ();
    void moveP();
    void moveL();
    void drag();
    void stop();
    void softStop();
    void run();
    void readState();
signals:
    void startReadState();
    void robotState(const st_SpaceCoordinate&space,const st_JointAngle&joint);
    void login();
    void logout();
    void lastMoveDoneBeforeCapture();
    void lastMoveDoneAfterCapture();


public slots:
    void loginResult(bool sucess);
    void logoutResult(bool sucess);
    void updateState(bool sucess,const st_SpaceCoordinate&space,const st_JointAngle&joint);
    void startIOWrite(const st_SpaceCoordinate&space,const st_JointAngle&joint);
    void startMove(const std::vector<Position>&,const std::vector<IOWrite>&,bool capture_cloud);

private:
    Ui::DeviceHarmo *ui;
    Console *console;
    PathTable *path_table;
    ProcessTree *process_tree;
    bool is_connected;
    bool is_poweron;
    bool is_enbled;
    bool is_run;
    bool is_drag;
    bool is_readstate;
    int current_io_num;
    int current_move_num;
    bool capture_cloud;
    QThread thread;
    QTimer *heart_beat;
    DeviceHarmoRead *Harmo_Read;
    std::vector<IOWrite> current_io;
    std::vector<Position> current_move;
    CTcpClientControlOperate Harmo_Control;
};

#endif // DEVICEHARMO_H
