#ifndef CUSTOMTREE_H
#define CUSTOMTREE_H
#include <QTreeWidget>

struct Index
{
    Index(){}
    Index(int r,int c):row(r),col(c){}
    bool operator !=(Index index){
        if(this->row!=index.row ||this->col!=index.col)
            return true;
        else
            return false;
    }
    bool operator ==(Index index){
        if(this->row==index.row && this->col==index.col)
            return true;
        else
            return false;
    }
    bool operator >(Index index){
        if(this->row>index.row)
            return true;
        else if(this->row==index.row&&this->col>index.col)
            return true;
        else
            return false;
    }
    bool operator <(Index index){
        if(this->row<index.row)
            return true;
        else if(this->row==index.row&&this->col<index.col)
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
    parent_first,
    child_first
};

class CustomTree: public QTreeWidget
{
    Q_OBJECT
public:
    explicit CustomTree(QWidget *parent = nullptr): QTreeWidget(parent){}

    std::vector<Index> getSelectedIndexs()
    {
        std::vector<Index> indexs;
        auto items = selectedItems();
        for (auto &i:items) {
            if(i->parent()==nullptr) {
                Index  index(indexOfTopLevelItem(i),-1);
                indexs.push_back(index);
            } else {
                QTreeWidgetItem* parent = i->parent();
                Index  index(indexOfTopLevelItem(parent),parent->indexOfChild(i));
                indexs.push_back(index);
            }
        }
        getSortedIndexs(parent_first,indexs);
        return indexs;
    }
    Index getSelectedIndex()
    {
        Index selectedIndex;
        if(getSelectedIndexs().empty()) return selectedIndex;
        else return getSelectedIndexs().front();
    }

    std::vector<Index> getClickedIndexs(QTreeWidgetItem *item)
    {
        std::vector<Index> indexs;
        if(item->parent()==nullptr) {
            Index  index(indexOfTopLevelItem(item),-1);
            indexs.push_back(index);
        } else {
            QTreeWidgetItem* parent = item->parent();
            Index  index(indexOfTopLevelItem(parent),parent->indexOfChild(item));
            indexs.push_back(index);
        }
        getSortedIndexs(parent_first,indexs);
        return indexs;
    }

    std::vector<Index> getCheckedIndexs()
    {
        std::vector<Index> indexs;
        for(int i=0;i<topLevelItemCount();i++) {
            QTreeWidgetItem* parent=topLevelItem(i);
            for(int j=0;j<parent->childCount();j++) {
                QTreeWidgetItem* child=parent->child(j);
                if(child->checkState(0)==Qt::Checked){
                    Index index(i,j);
                    indexs.push_back(index);
                }
            }
        }
        return indexs;
    }

    std::vector<Index> getAllIndexs()
    {
        std::vector<Index> indexs;
        for(int i=0;i<topLevelItemCount();i++) {
            QTreeWidgetItem* parent=topLevelItem(i);
            for(int j=0;j<parent->childCount();j++) {
                Index index(i,j);
                indexs.push_back(index);
            }
        }
        return indexs;
    }

protected:
    template <typename T>
    void addItem(int row,const T &data,const QString &parent_id,const QString &child_id,
                 const QIcon &parent_icon,const QIcon &child_icon,const bool &selected,
                 std::vector<std::vector<T>> &dataVec)
    {
        if(row==-1) {
            std::vector<T> temp;
            temp.push_back(data);
            dataVec.push_back(temp);
            QTreeWidgetItem* parent= new QTreeWidgetItem();
            QTreeWidgetItem* child= new QTreeWidgetItem();
            parent->setText(0, parent_id);
            parent->setIcon(0,parent_icon);
            child->setText(0,child_id);
            child->setIcon(0,child_icon);
            child->setCheckState(0, Qt::Checked);
            parent->setCheckState(0, Qt::Checked);
            parent->addChild(child);
            addTopLevelItem(parent);
            expandItem(parent);
            if(selected)
                setCurrentItem(child);
        } else {
            QTreeWidgetItem* child = new QTreeWidgetItem();
            child->setText(0,child_id);
            child->setIcon(0,child_icon);
            child->setCheckState(0, Qt::Checked);
            topLevelItem(row)->addChild(child);
            dataVec[row].push_back(data);
            expandItem(topLevelItem(row));
            if(selected)
                setCurrentItem(child);
        }
    }

    template <typename T>
    T getItem(const Index &index,const std::vector<std::vector<T>> &dataVec)
    {
        return  dataVec[index.row][index.col];
    }

    template <typename T>
    void updateItem(const Index &index,const T &update,std::vector<std::vector<T>> &dataVec)
    {
        dataVec[index.row][index.col]=update;
    }

    template <typename T>
    void removeItem(const Index &index,std::vector<std::vector<T>> &dataVec)
    {
        QTreeWidgetItem* parent = topLevelItem(index.row);
        if(parent->childCount()==1) {
            takeTopLevelItem(index.row);
            dataVec.erase(dataVec.begin() + index.row);
        }
        else {
            parent->removeChild(parent->child(index.col));
            dataVec[index.row].erase(dataVec[index.row].begin() + index.col);
        }
    }

    void setItemIcon(const Index &index,const QIcon &icon)
    {
        QTreeWidgetItem *parent=topLevelItem(index.row);
        if(index.col==-1)
            parent->setIcon(0,icon);
        else
            parent->child(index.col)->setIcon(0,icon);

    }

    void setItemText(const Index &index,const QString &name)
    {
        QTreeWidgetItem *parent=topLevelItem(index.row);
        if(index.col==-1)
            parent->setText(0,name);
        else
            parent->child(index.col)->setText(0,name);
    }

    void setItemChecked(const Index &index,bool checked)
    {
        QTreeWidgetItem *parent=topLevelItem(index.row);
        if(checked){
            parent->child(index.col)->setCheckState(0,Qt::Checked);
            int cnt=parent->childCount();
            int checked=0;
            for(int i=0;i<cnt;i++) {
                if(parent->child(i)->checkState(0)==Qt::Checked)
                    checked++;
            }
            if(cnt==checked)parent->setCheckState(0,Qt::Checked);
        }
        else {
            parent->child(index.col)->setCheckState(0,Qt::Unchecked);
            parent->setCheckState(0,Qt::PartiallyChecked);
        }
    }

    void setItemSelected(const Index &index,bool selected)
    {
        QTreeWidgetItem *parent=topLevelItem(index.row);
        parent->child(index.col)->setSelected(selected);
    }

    void renameItem(const Index &index,const QString &name)
    {
        QTreeWidgetItem *parent=topLevelItem(index.row);
        parent->child(index.col)->setText(0,name);
    }

    std::string getItemId(const Index &index)
    {
        QTreeWidgetItem* parent = topLevelItem(index.row);
        return parent->child(index.col)->text(0).toStdString();
    }

protected:
    void getSortedIndexs(sort_type type ,std::vector<Index> &indexs)
    {
        std::vector<Index> res;
        switch (type) {
        case ascending:
            std::sort(indexs.begin(), indexs.end(), [](Index &a, Index &b)->bool {
                return a < b ? true : false;
            });
            break;
        case descending:
            std::sort(indexs.begin(), indexs.end(), [](Index &a, Index &b)->bool {
                return a > b ? true : false;
            });
            break;
        case parent_first:
            for (auto& i : indexs) {
                if(i.col!=-1) {
                    Index temp(i.row,-1);
                    std::vector<Index>::iterator it=std::find(indexs.begin(),indexs.end(),temp);
                    if(it==indexs.end())
                        res.push_back(i);
                } else{
                    QTreeWidgetItem* parent =topLevelItem(i.row);
                    for (int j=0;j<parent->childCount();j++)
                        res.push_back( Index(i.row,j));
                }
            }
            indexs=res;
            break;
        case child_first:
            for (auto& i : indexs)
                if(i.col!=-1)
                    res.push_back(i);
            indexs=res;
            break;
        }
    }
};

#endif CUSTOMTREE_H
