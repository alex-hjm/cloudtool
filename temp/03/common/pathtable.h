#ifndef PATHTABLE_H
#define PATHTABLE_H
#include <QTableWidget>
#include <QLineEdit>
#include <QStringList>
#include <QHeaderView>
#include <QComboBox>
#include "common/console.h"
#include "common/tool.h"

enum command_type
{
    MoveJ=0,
    MoveP,
    MoveL,
    IO,
    Empty
};

struct Position
{
    Position(){}
    Position(float x_,float y_,float z_,float rx_,float ry_,float rz_):
        x(x_),y(y_),z(z_),rx(rx_),ry(ry_),rz(rz_){}
    bool operator !=(Position pos){
        if(this->x!=pos.x||this->y!=pos.y||this->z!=pos.z||this->rx!=pos.rx||this->ry!=pos.ry||this->rz!=pos.rz)
            return true;
        else
            return false;
    }
    bool operator ==(Position pos){
        if(this->x!=pos.x &&this->y!=pos.y&&this->z!=pos.z&&this->rx!=pos.rx&&this->ry!=pos.ry&&this->rz!=pos.rz)
            return true;
        else
            return false;
    }
//    bool operator >(Position pos){
//        if(this->x>pos.x &&this->y>pos.y&&this->z>pos.z&&this->rx>pos.rx&&this->ry>pos.ry&&this->rz>pos.rz)
//            return true;
//        else
//            return false;
//    }
//    bool operator <(Position pos){
//        if(this->x<pos.x &&this->y<pos.y&&this->z<pos.z&&this->rx<pos.rx&&this->ry<pos.ry&&this->rz<pos.rz)
//            return true;
//        else
//            return false;
//    }
    float x=0.0f;
    float y=0.0f;
    float z=0.0f;
    float rx=0.0f;
    float ry=0.0f;
    float rz=0.0f;
    float v=0.0f;
    command_type type=Empty;
};

struct IOWrite
{
    IOWrite(){}
    IOWrite(short num_,short io_type_,short count_,float value_):
        num(num_),io_type(io_type_),count(count_),value(value_){}
    bool operator !=(IOWrite io){
        if(this->num!=io.num||this->type!=io.type||this->count!=io.count||this->value!=io.value)
            return true;
        else
            return false;
    }
    bool operator ==(IOWrite io){
        if(this->num!=io.num&&this->io_type!=io.io_type&&this->count!=io.count&&this->value!=io.value)
            return true;
        else
            return false;
    }
    short num=0.0f;
    short io_type=0.0f;
    short count=0.0f;
    float value=0.0f;
    command_type type=Empty;
    Position pos;
};

class PathTable: public QTableWidget
{
public:
    PathTable(QWidget *parent = nullptr): QTableWidget(parent)
    {
        QStringList header;
        header<<"Type"<<"6D position";
        this->setColumnCount(2);
        this->setRowCount(0);
        this->setHorizontalHeaderLabels(header);
        this->horizontalHeader()->setVisible(true);
        this->horizontalHeader()->setDefaultSectionSize(55);
        this->horizontalHeader()->setStretchLastSection(true);
        this->verticalHeader()->setVisible(false);
        this->verticalHeader()->setDefaultSectionSize(25);
    }

    void addMove(const Position&pos)
    {
        QTableWidgetItem *type=new QTableWidgetItem();
        switch (pos.type) {
        case MoveJ:
            type->setIcon(QIcon(":/icon/resource/icon/format-node-curve.svg"));
            type->setText("MoveJ");
            type->setToolTip("MoveJ");
            break;
        case MoveP:
            type->setIcon(QIcon(":/icon/resource/icon/lines-connector.svg"));
            type->setText("MoveP");
            type->setToolTip("MoveP");
            break;
        case MoveL:
            type->setIcon(QIcon(":/icon/resource/icon/format-node-line.svg"));
            type->setText("MoveL");
            type->setToolTip("MoveL");
            break;
        }
        type->setFlags(type->flags() & (~Qt::ItemIsEditable));
        int row=this->rowCount();
        this->setRowCount(row+1);
        this->setItem(row,0,type);
        QString text=tr("%1 %2 %3 %4 %5 %6 %7").arg(pos.x).arg(pos.y).arg(pos.z).arg(pos.rx).arg(pos.ry).arg(pos.rz).arg(pos.v);
        QTableWidgetItem *item=new QTableWidgetItem();
        item->setText(text);
        item->setToolTip(text);
        this->setItem(row,1,item);
    }

    void addIO(const IOWrite&io)
    {
        QTableWidgetItem *type=new QTableWidgetItem();
        type->setIcon(QIcon(":/icon/resource/icon/network-connect.svg"));
        type->setText("IO");
        type->setToolTip("IO");
        type->setFlags(type->flags() & (~Qt::ItemIsEditable));
        int row=this->rowCount();
        this->setRowCount(row+1);
        this->setItem(row,0,type);
        QString text=tr("%1 %2 %3 %4").arg(io.num).arg(io.io_type+1).arg(io.count).arg(io.value);
        QTableWidgetItem *item=new QTableWidgetItem();
        item->setText(text);
        item->setToolTip(text);
        this->setItem(row,1,item);
    }

    void addCapture()
    {
        QTableWidgetItem *type=new QTableWidgetItem();
        type->setIcon(QIcon(":/icon/resource/icon/device.svg"));
        type->setText("Capture");
        type->setToolTip("Capture");
        type->setFlags(type->flags() & (~Qt::ItemIsEditable));
        int row=this->rowCount();
        this->setRowCount(row+1);
        this->setItem(row,0,type);
        QTableWidgetItem *item=new QTableWidgetItem("Start capture cloud");
        item->setToolTip(item->text());
        item->setFlags(item->flags() & (~Qt::ItemIsEditable));
        this->setItem(row,1,item);
    }

    int addPath(int num,const Eigen::Affine3f &position,float speed)
    {
        float x,y,z,rx,ry,rz;
        pcl::getTranslationAndEulerAngles(position,x,y,z,rx,ry,rz);
        QTableWidgetItem *type=new QTableWidgetItem();
        type->setIcon(QIcon(":/icon/resource/icon/format-convert-to-path.svg"));
        type->setText("path_"+QString::number(num));
        type->setToolTip("path_"+QString::number(num));
        int row=this->rowCount();
        this->setRowCount(row+1);
        this->setItem(row,0,type);
        QString text=tr("%1 %2 %3 %4 %5 %6 %7").arg(x).arg(y).arg(z).arg(rx/M_PI*180).arg(ry/M_PI*180).arg(rz/M_PI*180).arg(speed);
        QTableWidgetItem *item=new QTableWidgetItem();
        item->setText(text);
        item->setToolTip(text);
        this->setItem(row,1,item);
        return row;
    }

    int pathPointNum(int row)
    {
        QTableWidgetItem *item= this->item(row,0);
        QString num=item->text().remove("path_");
        return num.toInt();
    }

    void updatePath(int row,const Eigen::Affine3f &position)
    {
        QTableWidgetItem *item= this->item(row,1);
        QStringList textlist = item->text().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
        float speed=textlist[6].toFloat();
        float x,y,z,rx,ry,rz;
        pcl::getTranslationAndEulerAngles(position,x,y,z,rx,ry,rz);
        QString text=tr("%1 %2 %3 %4 %5 %6 %7").arg(x).arg(y).arg(z).arg(rx/M_PI*180).arg(ry/M_PI*180).arg(rz/M_PI*180).arg(speed);
        item->setText(text);
        item->setToolTip(text);
    }

    inline void clearCommand(int index)
    {
        this->removeRow(index);
    }

    void clearSelectCommand()
    {
        auto items=this->selectedItems();
        for(auto&i:items) {
            int index=this->row(i);
            this->clearCommand(index);
        }
    }

    std::vector<Position> getMoveDatas(bool before_capture)
    {
        std::vector<Position> moveDatas;
        int row=this->rowCount();
        int capture_row;
        for(int i=0;i<row;i++) {
            QTableWidgetItem *type= this->item(i,0);
            if(type->text().contains("Capture"))
                capture_row=i;
        }
        for(int i=0;i<row;i++) {
            QTableWidgetItem *type= this->item(i,0);
            QTableWidgetItem *item= this->item(i,1);
            if(!type->text().contains("Move")) continue;
            QStringList textlist = item->text().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
            if (textlist.size() != 7) continue;
            Position pos;
            if(type->text()=="MoveJ")
                pos.type=MoveJ;
            if(type->text()=="MoveP")
                pos.type=MoveP;
            if(type->text()=="MoveL")
                pos.type=MoveL;
            if(before_capture)
            {
                if(i<capture_row) {
                    pos.x=textlist[0].toFloat();
                    pos.y=textlist[1].toFloat();
                    pos.z=textlist[2].toFloat();
                    pos.rx=textlist[3].toFloat();
                    pos.ry=textlist[4].toFloat();
                    pos.rz=textlist[5].toFloat();
                    pos.v=textlist[6].toFloat();
                    moveDatas.push_back(pos);
                }
            }else {
                if(i>capture_row) {
                    pos.x=textlist[0].toFloat();
                    pos.y=textlist[1].toFloat();
                    pos.z=textlist[2].toFloat();
                    pos.rx=textlist[3].toFloat();
                    pos.ry=textlist[4].toFloat();
                    pos.rz=textlist[5].toFloat();
                    pos.v=textlist[6].toFloat();
                    moveDatas.push_back(pos);
                }
            }
        }
        return moveDatas;
    }

    std::vector<IOWrite> getIODatas(bool before_capture)
    {
        std::vector<IOWrite> ioDatas;
        int row=this->rowCount();
        int capture_row;
        for(int i=0;i<row;i++) {
            QTableWidgetItem *type= this->item(i,0);
            if(type->text().contains("Capture"))
                capture_row=i;
        }
        for(int i=0;i<row;i++) {
            QTableWidgetItem *type= this->item(i,0);
            QTableWidgetItem *item= this->item(i,1);
            if(!type->text().contains("IO")) continue;
            QStringList textlist = item->text().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
            if (textlist.size() != 4) continue;
            IOWrite io;
            io.type=IO;
            if(before_capture)
            {
                if(i<capture_row) {
                    for(int j=i;j>0;j--) {
                        QTableWidgetItem *type= this->item(j,0);
                        QTableWidgetItem *item= this->item(j,1);
                        if(!type->text().contains("Move"))continue;
                        QStringList textlist = item->text().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
                        if (textlist.size() != 7) continue;
                        if(type->text()=="MoveJ")
                            io.pos.type=MoveJ;
                        if(type->text()=="MoveP")
                            io.pos.type=MoveP;
                        if(type->text()=="MoveL")
                            io.pos.type=MoveL;
                        io.pos.x=textlist[0].toFloat();
                        io.pos.y=textlist[1].toFloat();
                        io.pos.z=textlist[2].toFloat();
                        io.pos.rx=textlist[3].toFloat();
                        io.pos.ry=textlist[4].toFloat();
                        io.pos.rz=textlist[5].toFloat();
                        io.pos.v=textlist[6].toFloat();
                    }
                    io.num=textlist[0].toShort();
                    io.io_type=textlist[1].toShort();
                    io.count=textlist[2].toShort();
                    io.value=textlist[3].toFloat();
                    ioDatas.push_back(io);
                }
            } else {
                if(i>capture_row) {
                    for(int j=i;j>capture_row;j--) {
                        QTableWidgetItem *type= this->item(j,0);
                        QTableWidgetItem *item= this->item(j,1);
                        if(!type->text().contains("Move"))continue;
                        QStringList textlist = item->text().split(QRegExp(",|\\s+"), QString::SkipEmptyParts);
                        if (textlist.size() != 7) continue;
                        if(type->text()=="MoveJ")
                            io.pos.type=MoveJ;
                        if(type->text()=="MoveP")
                            io.pos.type=MoveP;
                        if(type->text()=="MoveL")
                            io.pos.type=MoveL;
                        io.pos.x=textlist[0].toFloat();
                        io.pos.y=textlist[1].toFloat();
                        io.pos.z=textlist[2].toFloat();
                        io.pos.rx=textlist[3].toFloat();
                        io.pos.ry=textlist[4].toFloat();
                        io.pos.rz=textlist[5].toFloat();
                        io.pos.v=textlist[6].toFloat();
                    }
                    io.num=textlist[0].toShort();
                    io.io_type=textlist[1].toShort();
                    io.count=textlist[2].toShort();
                    io.value=textlist[3].toFloat();
                    ioDatas.push_back(io);
                }
            }
        }
        return ioDatas;
    }
};

#endif // PATHTABLE_H
