#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <QWidget>

namespace Ui {
class RobotControl;
}

class RobotControl : public QWidget
{
    Q_OBJECT

public:
    explicit RobotControl(QWidget *parent = nullptr);
    ~RobotControl();

private:
    Ui::RobotControl *ui;
};

#endif // ROBOTCONTROL_H
