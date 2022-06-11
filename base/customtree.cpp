/**
 * @file customtree.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#include "base/customtree.h"

namespace ct
{
    CustomTree::CustomTree(QWidget* parent) : QTreeWidget(parent)
    {
        this->setSelectionMode(QAbstractItemView::ExtendedSelection);
        connect(this, &CustomTree::itemSelectionChanged, this, &CustomTree::itemSelectionChangedEvent);
        connect(this, &CustomTree::itemClicked, this, &CustomTree::itemClickedEvent);
    }

    std::vector<Index> CustomTree::getSelectedIndexs()
    {
        std::vector<Index> indexs;
        auto items = selectedItems();
        for (auto& i : items)
            if (i->parent() != nullptr)
                indexs.push_back(Index(indexOfTopLevelItem(i->parent()), i->parent()->indexOfChild(i)));
        return indexs;
    }

    std::vector<Index> CustomTree::getCheckedIndexs()
    {
        std::vector<Index> indexs;
        for (int i = 0; i < topLevelItemCount(); i++)
        {
            QTreeWidgetItem* parent = topLevelItem(i);
            for (int j = 0; j < parent->childCount(); j++)
                if (parent->child(j)->checkState(0) == Qt::Checked)
                    indexs.push_back(Index(i, j));
        }
        return indexs;
    }

    std::vector<Index> CustomTree::getClickedIndexs(QTreeWidgetItem* item)
    {
        std::vector<Index> indexs;
        if (item->parent() == nullptr)
            for (int i = 0; i < item->childCount(); i++)
                indexs.push_back(Index(indexOfTopLevelItem(item), i));
        else
            indexs.push_back(Index(indexOfTopLevelItem(item->parent()), item->parent()->indexOfChild(item)));
        return indexs;
    }

    std::vector<Index> CustomTree::getAllIndexs()
    {
        std::vector<Index> indexs;
        for (int i = 0; i < topLevelItemCount(); i++)
            for (int j = 0; j < topLevelItem(i)->childCount(); j++)
                indexs.push_back(Index(i, j));
        return indexs;
    }

    bool CustomTree::indexIsValid(const Index& index)
    {
        return (index.row > -1) && (index.row < topLevelItemCount()) &&
            (index.col > -1) && (index.col < topLevelItem(index.row)->childCount());
    }

    void CustomTree::addItem(const Index& index, const QString& parent_id, const QString& child_id, bool selected)
    {
        if (index.row <= -1 || index.row > topLevelItemCount()) // parent item
        {
            QTreeWidgetItem* parent = new QTreeWidgetItem(this);
            QTreeWidgetItem* child = new QTreeWidgetItem(parent);
            parent->setText(0, parent_id);
            parent->setIcon(0, m_parent_icon);
            parent->setCheckState(0, Qt::Checked);
            child->setText(0, child_id);
            child->setIcon(0, m_child_icon);
            child->setCheckState(0, Qt::Checked);
            parent->addChild(child);
            addTopLevelItem(parent);
            expandItem(parent);
            if (selected)
                setCurrentItem(child);
        }
        else // child
        {
            QTreeWidgetItem* parent = topLevelItem(index.row);
            QTreeWidgetItem* child = new QTreeWidgetItem(parent);
            child->setIcon(0, m_child_icon);
            child->setText(0, child_id);
            child->setCheckState(0, Qt::Checked);
            if ((index.col > -1) && (index.col < topLevelItem(index.row)->childCount()))
                parent->insertChild(0, child);
            else
                parent->addChild(child);
            expandItem(parent);
            if (selected)
                setCurrentItem(child);
        }
    }

    void CustomTree::removeItem(const Index& index)
    {
        if (!indexIsValid(index))
            return;
        if (topLevelItem(index.row)->childCount() == 1)
            takeTopLevelItem(index.row);
        else
            topLevelItem(index.row)->takeChild(index.col);
    }

    QTreeWidgetItem* CustomTree::item(const Index& index)
    {
        if (!indexIsValid(index))
            return nullptr;
        if (index.col == -1)
            return topLevelItem(index.row);
        else
            return topLevelItem(index.row)->child(index.col);
    }

    Index CustomTree::index(const QString& text)
    {
        for (int i = 0; i < topLevelItemCount(); i++)
            for (int j = 0; j < topLevelItem(i)->childCount(); j++)
                if (topLevelItem(i)->child(j)->text(0) == text)
                    return Index(i, j);
        return Index(-1, -1);
    }

    void CustomTree::setItemChecked(const Index& index, bool checked)
    {
        if (!indexIsValid(index))
            return;
        QTreeWidgetItem* item = topLevelItem(index.row)->child(index.col);
        if (checked)
            item->setCheckState(0, Qt::Checked);
        else
            item->setCheckState(0, Qt::Unchecked);
        itemClickedEvent(item);
    }

    std::vector<Index> CustomTree::getSortedIndexs(sort_type type, const std::vector<Index>& indexs)
    {
        std::vector<Index> res;
        switch (type)
        {
        case ASCENDING:
            res = indexs;
            std::sort(res.begin(), res.end(), [](Index& a, Index& b) -> bool
                      { return (a.row < b.row) || (a.row == b.row && a.col < b.col) ? true : false; });
            break;
        case DESCENDING:
            res = indexs;
            std::sort(res.begin(), res.end(), [](Index& a, Index& b) -> bool
                      { return (a.row > b.row) || (a.row == b.row && a.col > b.col) ? true : false; });
            break;
        case PARENTFIRST:
            for (auto& i : indexs)
            {
                if (i.col != -1)
                {
                    if (std::find(indexs.begin(), indexs.end(), Index(i.row, -1)) == indexs.end())
                        res.push_back(i);
                }
                else
                {
                    QTreeWidgetItem* parent = topLevelItem(i.row);
                    for (int j = 0; j < parent->childCount(); j++)
                        res.push_back(Index(i.row, j));
                }
            }
            break;
        case CHILDFIRST:
            for (auto& i : indexs)
                if (i.col != -1)
                    res.push_back(i);
            break;
        }
        return res;
    }

    void CustomTree::itemSelectionChangedEvent()
    {
        auto items = selectedItems();
        for (auto& item : items)
        {
            if (item->parent() == nullptr)
                for (int i = 0; i < item->childCount(); i++)
                    item->child(i)->setSelected(true);
            else
            {
                int i, cout = item->parent()->childCount();
                for (i = 0; i < cout; i++)
                    if (!item->parent()->child(i)->isSelected())
                        break;
                if (i == cout)
                    item->parent()->setSelected(true);
            }
        }
    }

    void CustomTree::itemClickedEvent(QTreeWidgetItem* item, int)
    {
        if (item->parent() == nullptr)
        {
            for (int i = 0; i < item->childCount(); i++)
            {
                if (item->checkState(0) == Qt::Checked)
                    item->child(i)->setCheckState(0, Qt::Checked);
                else if (item->checkState(0) == Qt::Unchecked)
                    item->child(i)->setCheckState(0, Qt::Unchecked);
            }
        }
        else
        {
            QTreeWidgetItem* parent = item->parent();
            if (item->checkState(0) == Qt::Checked)
            {
                int i, cout = parent->childCount();
                for (i = 0; i < cout; i++)
                    if (parent->child(i)->checkState(0) == Qt::Unchecked)
                        break;
                if (i == cout)
                    parent->setCheckState(0, Qt::Checked);
                else
                    parent->setCheckState(0, Qt::PartiallyChecked);
            }
            else
            {
                int i, cout = parent->childCount();
                for (i = 0; i < cout; i++)
                    if (parent->child(i)->checkState(0) == Qt::Checked)
                        break;
                if (i == cout)
                    parent->setCheckState(0, Qt::Unchecked);
                else
                    parent->setCheckState(0, Qt::PartiallyChecked);
            }
        }
    }

} // namespace pca