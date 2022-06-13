#include "processtree.h"

ProcessTree::ProcessTree(QWidget *parent) : CustomTree(parent),process_enable(false)
{
    this->setHeaderLabel("Pipeline Browser");
    this->header()->setDefaultAlignment(Qt::AlignCenter);
    this->setItemsExpandable(false);
    connect(this,&ProcessTree::itemClicked,this,&ProcessTree::updateScreen);
}

void ProcessTree::insertProcess(const Process::Ptr &process)
{
    Process::Ptr selectedProcess=this->getSelectedProcess();
    Index index=this->getSelectedIndex();
    if(selectedProcess->type==process_empty){
        this->addItem<Process::Ptr>(-1,process,process->id.c_str(),process->icon,false,process_vec);
    } else if(selectedProcess->type!=process->type) {
        this->addItem<Process::Ptr>(index.row,process,process->id.c_str(),process->icon,false,process_vec);
    } else {
        QMessageBox message_box(QMessageBox::NoIcon,"",tr("The same process exists"),QMessageBox::NoButton,this,
                                Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);
        message_box.addButton(tr("Replace"), QMessageBox::ActionRole)->setDefault(true);
        message_box.addButton(tr("Insert"),QMessageBox::ActionRole);
        message_box.addButton(QMessageBox::Cancel);
        int k=message_box.exec();
        if(k==QMessageBox::Cancel){
            return;
        } else if(k==0) {
            *selectedProcess=*process;
        } else if(k==1) {
            this->addItem<Process::Ptr>(index.row,process,process->id.c_str(),process->icon,false,process_vec);
        }
    }
}

void ProcessTree::clearProcess(const Index &index)
{
    if(index.row==-1||index.col==-1) return;
    this->removeItem<Process::Ptr>(index,process_vec);
    for(auto &i:process_vec)
        std::vector<Process::Ptr>(i).swap(i);
    std::vector<std::vector<Process::Ptr>>(process_vec).swap(process_vec);
}

void ProcessTree::clearSelectedProcesses()
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    this->getSortedIndexs(descending,indexs);
    for(auto &i:indexs)
        this->clearProcess(i);
}

void ProcessTree::clearAllProcesses()
{
    this->clear();
    this->process_vec.clear();
    for(auto &i:process_vec)
        std::vector<Process::Ptr>().swap(i);
    std::vector<std::vector<Process::Ptr>>().swap(process_vec);
}

void ProcessTree::setProcessChecked(const Index &index, bool checked)
{
    if(index.row==-1||index.col==-1) return;
    this->setItemChecked(index,checked);
    Process::Ptr process=this->getItem<Process::Ptr>(index,process_vec);
    process->is_checked=checked;
}

void ProcessTree::setSelectedProcessesChecked(bool checked)
{
    std::vector<Index> indexs=this->getSelectedIndexs();
    for(auto &i:indexs)
        this->setProcessChecked(i,checked);
}

void ProcessTree::showSelectedProcessesDetail()
{
    Process::Ptr selectedProcess=this->getSelectedProcess();
    QDialog *detail_dialog=new QDialog(this);
    detail_dialog->setAttribute(Qt::WA_DeleteOnClose);
    detail_dialog->setWindowFlags(Qt::FramelessWindowHint|Qt::Dialog);
    QVBoxLayout *layout=new QVBoxLayout(detail_dialog);
    switch (selectedProcess->type) {
    case process_empty:
        break;
    case process_filter:
        layout->addWidget(new QLabel(selectedProcess->id.c_str()));
        for(size_t i=2;i<selectedProcess->value.size();i++) {
           layout->addWidget(new QLabel(tr("value[%1]: %2").arg(i).arg(selectedProcess->value[i])));
        }
        break;
    case process_color:
        layout->addWidget(new QLabel(selectedProcess->id.c_str()));
        for(size_t i=0;i<selectedProcess->value.size();i++) {
           layout->addWidget(new QLabel(tr("value[%1]: %2").arg(i).arg(selectedProcess->value[i])));
        }
        break;
    case process_coordinate:
        layout->addWidget(new QLabel(selectedProcess->id.c_str()));
        layout->addWidget(new QLabel(tr("value[0]: %1").arg(selectedProcess->value[0])));
        layout->addWidget(new QLabel("matrix:"));
        for(size_t i=0;i<4;i++) {
           layout->addWidget(new QLabel(tr("%1 %2 %3 %4").arg(selectedProcess->affine(i,0)).
                                        arg(selectedProcess->affine(i,1))
                                        .arg(selectedProcess->affine(i,2)).
                                        arg(selectedProcess->affine(i,3))));
        }
        break;
    case process_transformation:
        layout->addWidget(new QLabel(selectedProcess->id.c_str()));
        layout->addWidget(new QLabel("matrix:"));
        for(size_t i=0;i<4;i++) {
           layout->addWidget(new QLabel(tr("%1 %2 %3 %4").arg(selectedProcess->affine(i,0)).
                                        arg(selectedProcess->affine(i,1))
                                        .arg(selectedProcess->affine(i,2)).
                                        arg(selectedProcess->affine(i,3))));
        }
        break;
    case process_registration:
        layout->addWidget(new QLabel(selectedProcess->id.c_str()));
        for(size_t i=1;i<selectedProcess->value.size();i++) {
           layout->addWidget(new QLabel(tr("value[%1]: %2").arg(i).arg(selectedProcess->value[i])));
        }
        break;
        break;
    }
    QPushButton *btn_close=new QPushButton("OK",detail_dialog);
    connect(btn_close,&QPushButton::clicked,[detail_dialog]{detail_dialog->close();});
    layout->addWidget(btn_close);
    detail_dialog->setLayout(layout);
    detail_dialog->show();
}

Process::Ptr ProcessTree::getSelectedProcess()
{
    Process::Ptr selectedProcess(new Process);
    if(getSelectedProcesses().empty())return selectedProcess;
    else return this->getSelectedProcesses().front();
}

std::vector<Process::Ptr> ProcessTree::getSelectedProcesses()
{
    std::vector<Index> indexs=getSelectedIndexs();
    std::vector<Process::Ptr> selectedProcesses;
    for (auto &i:indexs)
        selectedProcesses.push_back(this->getItem<Process::Ptr>(i,process_vec));
    return selectedProcesses;
}

std::vector<Process::Ptr> ProcessTree::getAllProcess()
{
    std::vector<Index> indexs=getAllIndexs();
    std::vector<Process::Ptr> allProcesses;
    for (auto &i:indexs)
        allProcesses.push_back(this->getItem<Process::Ptr>(i,process_vec));
    return allProcesses;
}

void ProcessTree::setProcessEnable(bool enable)
{
    process_enable=enable;
}

void ProcessTree::setExtendedSelection(bool enable)
{
    if(enable)
        this->setSelectionMode(QAbstractItemView::ExtendedSelection);
    else
        this->setSelectionMode(QAbstractItemView::SingleSelection);
}

void ProcessTree::updateScreen(QTreeWidgetItem * item, int)
{
    Qt::CheckState CheckState=item->checkState(0);
    std::vector<Index> indexs=this->getClickedIndexs(item);
    for(auto &i:indexs)
        if(CheckState==Qt::Checked)
            this->setProcessChecked(i,true);
        else
            this->setProcessChecked(i,false);
}

void ProcessTree::mousePressEvent(QMouseEvent *event)
{
    QModelIndex indexSelect = indexAt(event->pos());
    if(indexSelect.row() == -1)
        setCurrentIndex(indexSelect);
    if(event->button() == Qt::RightButton && indexSelect.row() != -1){
        QTreeWidgetItem *item=this->itemFromIndex(indexSelect);
        if(item->isSelected()) {
            QMenu menu;
            QAction *clearAction = menu.addAction(QIcon(":/icon/resource/icon/close-copy.svg"),"clear");
            connect(clearAction,&QAction::triggered,this,&ProcessTree::clearSelectedProcesses);
            if(item->checkState(0)==Qt::Checked) {
                QAction *hideAction = menu.addAction(QIcon(":/icon/resource/icon/view-hidden.svg"),"hide");
                connect(hideAction,&QAction::triggered,this,[=]{this->setSelectedProcessesChecked(false);});
            } else if(item->checkState(0)==Qt::Unchecked||item->checkState(0)==Qt::PartiallyChecked) {
                QAction *showAction = menu.addAction(QIcon(":/icon/resource/icon/view-show.svg"),"show");
                connect(showAction,&QAction::triggered,this,[=]{this->setSelectedProcessesChecked(true);});
            }
            QAction *detailAction = menu.addAction(QIcon(":/icon/resource/icon/view-choose.svg"),"detail");
            connect(detailAction,&QAction::triggered,this,&ProcessTree::showSelectedProcessesDetail);
            menu.exec(event->globalPos());
            event->accept();
        }
    }
    return CustomTree::mousePressEvent(event);
}
