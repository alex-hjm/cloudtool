#ifndef CUSTOMTREE_H
#define CUSTOMTREE_H
#include <QTreeWidget>
#include <QTableWidget>
#include <QMouseEvent>
#include <QHeaderView>

struct Index
{
    Index(){}
    Index(int r,int c):row(r),col(c){}
    bool operator !=(Index index){
        if((this->row!=index.row )|(this->col!=index.col))
            return true;
        else
            return false;
    }
    bool operator ==(Index index){
        if((this->row==index.row && this->col==index.col))
            return true;
        else
            return false;
    }
    int row=-1;
    int col=-1;
};

enum sort_type
{
    ascending=0,
    descending,
    parentFirst,
};

class CustomTree: public QTreeWidget
{
    Q_OBJECT
public:
    explicit CustomTree(QWidget *parent = nullptr): QTreeWidget(parent) {
        connect(this,&CustomTree::itemClicked,this,&CustomTree::updateTree);
    }

    Index getSelectedIndex()
    {
        Index index;
        if(this->getSelectedIndexs().size()<=0)
            return index;
        else
            return this->getSelectedIndexs()[0];
    }

    std::vector<Index> getClickedIndexs(QTreeWidgetItem *item)
    {
        std::vector<Index> indexs;
        Index  index;
        if(item->parent()==nullptr) {
            index.row = this->indexOfTopLevelItem(item);
            index.col = -1;
        } else {
            QTreeWidgetItem* parent = item->parent();
            index.row = this->indexOfTopLevelItem(parent);
            index.col = parent->indexOfChild(item);
        }
        indexs.push_back(index);
        return this->getSortedIndexs(parentFirst,indexs);
    }

    std::vector<Index> getSelectedIndexs()
    {
        std::vector<Index> indexs;
        auto items = this->selectedItems();
        for (int i=0;i<items.size();i++) {
            Index  index;
            if(items[i]->parent()==nullptr) {
                index.row = this->indexOfTopLevelItem(items[i]);
                index.col = -1;
            }
            else {
                QTreeWidgetItem* parent = items[i]->parent();
                index.row = this->indexOfTopLevelItem(parent);
                index.col = parent->indexOfChild(items[i]);
            }
            indexs.push_back(index);
        }
        return this->getSortedIndexs(parentFirst,indexs);
    }

    std::vector<Index> getCheckedIndexs()
    {
        std::vector<Index> indexs;
        int parent_num=this->topLevelItemCount();
        for(int i=0;i<parent_num;i++) {
            QTreeWidgetItem* parent=this->topLevelItem(i);
            int child_num=parent->childCount();
            for(int j=0;j<child_num;j++) {
                QTreeWidgetItem* child=parent->child(j);
                if(child->checkState(0)==Qt::Checked){
                    Index index(i,j);
                    indexs.push_back(index);
                }
            }
        }
        return this->getSortedIndexs(parentFirst,indexs);
    }

    std::vector<Index> getAllIndexs()
    {
        std::vector<Index> indexs;
        int parentCount=this->topLevelItemCount();
        for(int i=0;i<parentCount;i++) {
            QTreeWidgetItem* parent=this->topLevelItem(i);
            int childCount=parent->childCount();
            for(int j=0;j<childCount;j++) {
                Index index(i,j);
                indexs.push_back(index);
            }
        }
        return indexs;
    }

protected:
    std::vector<Index> getSortedIndexs(sort_type type ,std::vector<Index> &indexs)
    {
        std::vector<Index> res;
        switch (type) {
        case ascending:
            std::sort(indexs.begin(), indexs.end(), [](Index a, Index b)->bool {
                return a.row < b.row ? true : (a.row == b.row ? a.col < b.col : false);
            });
            res =indexs;
            break;
        case descending://descending
            std::sort(indexs.begin(), indexs.end(), [](Index a, Index b)->bool {
                return a.row > b.row ? true : (a.row == b.row ? a.col > b.col : false);
            });
            res =indexs;
            break;
        case parentFirst://ParentFirst
            for (int i=0;i<indexs.size();i++) {
                if(indexs[i].col!=-1) {
                    for(int j=0;j<indexs.size();j++) {
                        if(indexs[i].row==indexs[j].row && indexs[j].col==-1) {
                            break;
                        }
                        if(j==indexs.size()-1) {
                            res.push_back(indexs[i]);
                            break;
                        }
                    }
                }
                else {
                    QTreeWidgetItem* parent =this->topLevelItem(indexs[i].row);
                    for (int j=0;j<parent->childCount();j++) {
                        Index temp(indexs[i].row,j);
                        res.push_back(temp);
                    }
                }
            }
            break;
        }
        return res;
    }

    template <typename T>
    void addItem(int row,const T &data,const QString &parentId,const QString &childId,
                 const QIcon &parentIcon,const QIcon &childIcon,const bool &selected,
                 std::vector<std::vector<T>> &dataVec)
    {
        if(row==-1){
            std::vector<T> temp;
            temp.push_back(data);
            dataVec.push_back(temp);
            QTreeWidgetItem* item = new QTreeWidgetItem();
            item->setText(0, parentId);
            item->setCheckState(0, Qt::Checked);
            QTreeWidgetItem* child = new QTreeWidgetItem();
            child->setText(0,childId);
            child->setCheckState(0, Qt::Checked);
            item->setIcon(0,parentIcon);
            child->setIcon(0,childIcon);
            item->addChild(child);
            this->addTopLevelItem(item);
            if(selected)
                this->setCurrentItem(child);
            this->expandItem(item);
        } else if(row >= 0 && row < dataVec.size()) {
            QTreeWidgetItem* child = new QTreeWidgetItem();
            child->setText(0, childId);
            child->setCheckState(0, Qt::Checked);
            dataVec[row].push_back(data);
            child->setIcon(0,childIcon);
            this->topLevelItem(row)->addChild(child);
            if(selected)
                this->setCurrentItem(child);
            this->expandItem(this->topLevelItem(row));
        }
    }

    template <typename T>
    void updateItem(const Index &index,T &update,std::vector<std::vector<T>> &dataVec)
    {
        dataVec[index.row][index.col]=update;
    }

    template <typename T>
    void removeItem(const Index &index,std::vector<std::vector<T>> &dataVec)
    {
        QTreeWidgetItem* parent = this->topLevelItem(index.row);
        if(parent->childCount()==1) {
            this->takeTopLevelItem(index.row);
            dataVec.erase(dataVec.begin() + index.row);
        }
        else {
            parent->removeChild(parent->child(index.col));
            dataVec[index.row].erase(dataVec[index.row].begin() + index.col);
        }
    }

    void setItemChecked(const Index &index,bool checked)
    {
        QTreeWidgetItem *parent=this->topLevelItem(index.row);
        QTreeWidgetItem *child=parent->child(index.col);
        if(checked)
            child->setCheckState(0,Qt::Checked);
        else
            child->setCheckState(0,Qt::Unchecked);
    }

    void setItemSelected(const Index &index,bool selected)
    {
        QTreeWidgetItem *parent=this->topLevelItem(index.row);
        QTreeWidgetItem *child=parent->child(index.col);
        child->setSelected(selected);
    }

    void renameItem(const Index &index,const QString &name)
    {
        QTreeWidgetItem *parent=this->topLevelItem(index.row);
        QTreeWidgetItem *child=parent->child(index.col);
        child->setText(0,name);
    }


    template <typename T>
    T getData(const Index &index,const std::vector<std::vector<T>> &dataVec)
    {
        return  dataVec[index.row][index.col];
    }

    template <typename T>
    std::string getDataId(const Index &index,const std::vector<std::vector<T>> &dataVec)
    {
        return  dataVec[index.row][index.col].id;
    }

    template <typename T>
    std::vector<T> getSelectedDatas(std::vector<std::vector<T>> &dataVec)
    {
        std::vector<Index> indexs=getSelectedIndexs();
        std::vector<T> selectedDatas;
        for (int i=0;i<indexs.size();i++) {
            selectedDatas.push_back(this->getData(indexs[i],dataVec));
        }
        return selectedDatas;
    }

    template <typename T>
    std::vector<std::string> getSelectedDatasId(std::vector<std::vector<T>> &dataVec)
    {
        std::vector<Index> indexs=getSelectedIndexs();
        std::vector<std::string> selectedDatasID;
        for (int i=0;i<indexs.size();i++) {
            selectedDatasID.push_back(this->getDataId(indexs[i],dataVec));
        }
        return selectedDatasID;
    }

    template <typename T>
    std::vector<T> getClickedDatas(QTreeWidgetItem *item,std::vector<std::vector<T>> &dataVec)
    {
        std::vector<Index> indexs=getClickedIndexs(item);
        std::vector<T> clickedDatas;
        for (int i=0;i<indexs.size();i++) {
            clickedDatas.push_back(this->getData(indexs[i],dataVec));
        }
        return clickedDatas;
    }

    template <typename T>
    std::vector<T> getCheckedDatas(std::vector<std::vector<T>> &dataVec)
    {
        std::vector<Index> indexs=this->getCheckedIndexs();
        std::vector<T> CheckedDatas;
        for (int i=0;i<indexs.size();i++) {
            CheckedDatas.push_back(this->getData(indexs[i],dataVec));
        }
        return CheckedDatas;
    }

    template <typename T>
    std::vector<T> getAllDatas(std::vector<std::vector<T>> &dataVec)
    {
        std::vector<T> allDatas;
        for (int i=0;i<dataVec.size();i++) {
            allDatas.insert(allDatas.end(),dataVec[i].begin(),dataVec[i].end());
        }
        return allDatas;
    }

public slots:
    void updateTree(QTreeWidgetItem * item, int )
    {
        Qt::CheckState CheckState=item->checkState(0);
        int num=item->childCount();
        if (num>0) {
            for(int i=0;i<num;i++) {
                if(CheckState==Qt::Checked) {
                    if(item->child(i)->checkState(0)==Qt::Unchecked)
                        item->child(i)->setCheckState(0,Qt::Checked);
                } else  {
                    if(item->child(i)->checkState(0)==Qt::Checked)
                        item->child(i)->setCheckState(0,Qt::Unchecked);
                }
            }
        } else {
            QTreeWidgetItem* parent = item->parent();
            if(CheckState==Qt::Unchecked)
                parent->setCheckState(0,Qt::PartiallyChecked);
            else {
                int cnt=parent->childCount();
                int checked=0;
                for(int i=0;i<cnt;i++) {
                    if(parent->child(i)->checkState(0)==Qt::Checked)
                        checked++;
                }
                if(cnt==checked)parent->setCheckState(0,Qt::Checked);
            }
        }
    }

protected:
    void mousePressEvent(QMouseEvent* event)
    {
        QModelIndex indexSelect = indexAt(event->pos());
        if(indexSelect.row() == -1)
            setCurrentIndex(indexSelect);
        return QTreeWidget::mousePressEvent(event);
    }

    //private:
    //    QSize sizeHint() const
    //    { return QSize(226, 207);}

};

class PropertyTable: public QTableWidget
{
    Q_OBJECT
public:
    explicit PropertyTable(QWidget *parent = nullptr): QTableWidget(parent) {
        this->resizeColumnsToContents();
        this->verticalHeader()->setVisible(false);
        this->horizontalHeader()->setVisible(true);
        this->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        this->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    }
    //private:
    //    QSize sizeHint() const
    //    { return QSize(226, 266);}
};

#endif // CUSTOMTREE_H
