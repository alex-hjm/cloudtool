#ifndef PROCESSTREE_H
#define PROCESSTREE_H
#include <QMessageBox>
#include <QPushButton>
#include <QHeaderView>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QDialog>
#include <QMenu>
#include <QLabel>
#include <pcl/common/transforms.h>
#include "customtree.h"
#include "cloud.h"
#include "console.h"
#include "pathtable.h"

enum process_type
{
    process_empty=0,
    process_color,
    process_filter,
    process_transformation,
    process_coordinate,
    process_registration,
    process_device
};

struct Process
{
    std::string id;
    QIcon icon;
    bool is_checked=false;
    process_type type=process_empty;
    std::vector<float> value;
    Eigen::Affine3f affine=Eigen::Affine3f::Identity();
    using Ptr = std::shared_ptr<Process>;
    using ConstPtr = std::shared_ptr<const Process>;
};

class ProcessTree: public CustomTree
{
    Q_OBJECT
public:
    explicit ProcessTree(QWidget *parent = nullptr);
    void insertProcess(const Process::Ptr &process);
    void clearProcess(const Index &index);
    void clearSelectedProcesses();
    void clearAllProcesses();
    void setProcessChecked(const Index &index,bool checked);
    void setSelectedProcessesChecked(bool checked);
    void showSelectedProcessesDetail();

    Process::Ptr getSelectedProcess();
    std::vector<Process::Ptr> getSelectedProcesses();
    std::vector<Process::Ptr> getAllProcess();

    void setProcessEnable(bool enable);
    void setExtendedSelection(bool enable);
    inline bool enable(){return process_enable;}

//zhisensor
signals:
    void startCapture();
    void captureCloud(Cloud::Ptr &);
    void startMove(const std::vector<Position>&,const std::vector<IOWrite>&,bool capture_cloud);
    void lastMoveDoneBeforeCapture();
    void lastMoveDoneAfterCapture();

//harmo
    void moveJ();
    void moveP();
    void moveL();
    void ioWrite();

public slots:
    void updateScreen(QTreeWidgetItem *, int);

protected:
    void mousePressEvent(QMouseEvent *event);

private:
    bool process_enable;
    std::vector<std::vector<Process::Ptr>> process_vec;
};

#endif // PROCESSTREE_H
