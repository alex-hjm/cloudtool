#include "processtree.h"

ProcessTree::ProcessTree(QWidget *parent) : CustomTree(parent),
    processEnable(false)
{
    this->setHeaderLabel("Pipeline Browser");
    this->header()->setDefaultAlignment(Qt::AlignHCenter);
    this->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Expanding);
    this->setItemsExpandable(false);
    connect(this,&ProcessTree::itemSelectionChanged,this,&ProcessTree::updatePanel);
    connect(this,SIGNAL(itemClicked(QTreeWidgetItem *, int)),this,SLOT(updateScreen(QTreeWidgetItem *, int)));
}

void ProcessTree::setExtendedSelection(const bool &enable)
{
    if(enable)
        this->setSelectionMode(QAbstractItemView::ExtendedSelection);
    else
        this->setSelectionMode(QAbstractItemView::SingleSelection);

}

void ProcessTree::insertItem(Process &process)
{
    selectedProcess=this->getSelectedProcess();
    index=this->getSelectedIndex();
    if(selectedProcess.isEmpty){
        this->addItem<Process>(-1,process,process.id.c_str(),process.icon.c_str(),prcessVec);
        return;
    }
    else if(selectedProcess.type!=process.type){
        this->addItem<Process>(index.row,process,process.id.c_str(),process.icon.c_str(),prcessVec);
        return;
    }
    else{
        int k= QMessageBox::information(this,"",tr("The same process exists"),tr("replace"),tr("insert"),tr("cancel"));
        switch (k){
        case 0:
            this->updateItem(index,process);
            break;
        case 1:
            this->addItem<Process>(index.row,process,process.id.c_str(),process.icon.c_str(),prcessVec);
            break;
        case 2:
            break;
        }
        return;
    }

}

void ProcessTree::updateItem(const Index &index,const Process &updateProcess)
{
    this->updateSelectedItem<Process>(index,updateProcess,prcessVec);
}

void ProcessTree::loadItems()
{

}

void ProcessTree::saveItems()//TODO:output->config.ini
{
//    if(prcessVec.size()<=0) {
//        console->warning("Please add a process");
//        return;
//    }
//    QFileInfo fileInfo = QFileDialog::getSaveFileName(this, tr ("Save setting"),"./setting", "ini(*.ini)");
//    QTextCodec *pcode = QTextCodec::codecForName("GB2312");
//    QString file_name=pcode->fromUnicode(fileInfo.filePath()).data();
//    if(!file_name.endsWith(".ini")) {
//        file_name.append(".ini");
//    }
//    QFile setting(file_name);
//    if (setting.open(QIODevice::ReadWrite | QIODevice::Text))
//    {

//        setting.close();
//    }
}

void ProcessTree::clearItem()
{
    this->removeSelectedItems<Process>(prcessVec);
}

void ProcessTree::clearAllItem()
{
    this->prcessVec.clear();
    this->clear();
}

Process ProcessTree::getSelectedProcess()
{
    Process selectedProcess;
    if(this->getSelectedDatas<Process>(prcessVec).size()<=0)
        return selectedProcess;
    else
        return this->getSelectedDatas<Process>(prcessVec)[0];
}

std::vector<Process> ProcessTree::getAllProcess()
{
    return this->getAllDatas(prcessVec);
}

void ProcessTree::processChanged(bool state)
{
    processEnable=state;
}

void ProcessTree::updatePanel()
{
    selectedProcess=this->getSelectedProcess();
    index=this->getSelectedIndex();
    auto items = this->selectedItems();
    if(items.size()<=0)return;
    if(selectedProcess.isEmpty) return;
    emit processData(selectedProcess);
}

void ProcessTree::updateScreen(QTreeWidgetItem * item, int)
{
    clickedProcess=this->getclickedDatas<Process>(item,prcessVec)[0];
    Qt::CheckState CheckState=item->checkState(0);
    index.row=this->indexOfTopLevelItem(item);
    index.col=0;
    if(CheckState==Qt::Checked){
        clickedProcess.isHidden=false;
        this->updateItem(index,clickedProcess);
    }
    else {
        clickedProcess.isHidden=true;
        this->updateItem(index,clickedProcess);
    }


}
