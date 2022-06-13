#ifndef TEMPLATE_H
#define TEMPLATE_H

#include <QDialog>
#include "common/cloud.h"

namespace Ui {
class Template;
}

class Template : public QDialog
{
    Q_OBJECT

public:
    explicit Template(QWidget *parent = nullptr);
    ~Template();
    void setCommand();
    void setAllPath(int size);
    void setTemplate(const std::vector<Cloud::Ptr> &all_clouds);
    void setTemplateWithDevice(const std::vector<Cloud::Ptr> &all_clouds);
    void setExtendedSelection(bool enable);
    void accept();
signals:
    void getIndex(int);
    void getTemplate(const std::vector<int> &);
    void getPath(int index,float speed);
private:
    Ui::Template *ui;
};

#endif // TEMPLATE_H
