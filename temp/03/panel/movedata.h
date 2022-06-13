#ifndef MOVEDATA_H
#define MOVEDATA_H

#include <QDialog>
#include "common/pathtable.h"

namespace Ui {
class MoveData;
}

class MoveData : public QDialog
{
    Q_OBJECT

public:
    explicit MoveData(QWidget *parent = nullptr);
    ~MoveData();
    void setMoveJ();
    void setMoveP();
    void setMoveL();
    void setIO();
    void add();
    void move();
signals:
    void addMoveData(const Position&pos);
    void addIoData(const IOWrite&pos);
    void moveData(const Position&pos);
    void ioData(const IOWrite&io);
private:
    Ui::MoveData *ui;
    int current_type;

};

#endif // MOVEDATA_H
