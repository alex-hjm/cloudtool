#ifndef PROCESSTREE_H
#define PROCESSTREE_H
#include <QWidget>
#include <QMessageBox>
#include <QTextStream>
#include <QTextCodec>
#include <QInputDialog>
#include <QFileDialog>

#include "customtree.h"
#include "console.h"

class ProcessTree: public CustomTree
{
    Q_OBJECT
public:
    explicit ProcessTree(QWidget *parent = nullptr);

    Console *console;

    bool processEnable;
    void setExtendedSelection(const bool &enable);
    void insertItem(Process &process);
    void updateItem(const Index &index,const Process &updateProcess);
    void loadItems();
    void saveItems();
    void clearItem();
    void clearAllItem();

    Process getSelectedProcess();
    std::vector<Process> getAllProcess();

signals:
    void processData(const Process &);
    void deviceCapture();
    void captureState(bool,bool);
    void captureCloud(const CloudXYZRGBN::Ptr &);

public slots:
    void processChanged(bool);
    void updatePanel();
    void updateScreen(QTreeWidgetItem *, int);

private:
    Index index;
    Process selectedProcess;
    Process clickedProcess;
    std::vector<std::vector<Process>> prcessVec;

};

#endif // PROCESSTREE_H
