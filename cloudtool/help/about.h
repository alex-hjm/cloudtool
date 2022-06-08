/**
 * @file about.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-09
 */
#ifndef CT_HELP_ABOUT_H
#define CT_HELP_ABOUT_H

#include <QDialog>

namespace Ui
{
    class About;
}

class About : public QDialog
{
    Q_OBJECT

public:
    explicit About(QWidget *parent = nullptr);
    ~About();

private:
    Ui::About *ui;
};

#endif // CT_HELP_ABOUT_H
