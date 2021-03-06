/**
 * @file customtree.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CUSTOMTREE_H
#define CT_BASE_CUSTOMTREE_H



#include "base/exports.h"

#include "base/cloudview.h"
#include "base/console.h"

#include <QTreeWidget>
#include <QTableWidget>
#include <QProgressBar>

namespace ct
{
    enum sort_type
    {
        ASCENDING,   // 递增
        DESCENDING,  // 递减
        PARENTFIRST, // 父类优先
        CHILDFIRST   // 子类优先
    };

    struct Index
    {
        Index() {}
        Index(int r, int c) : row(r), col(c) {}
        bool operator==(const Index& index) const
        {
            return (this->row == index.row) && (this->col == index.col);
        }
        bool operator!=(const Index& index) const
        {
            return !(*this == index);
        }
        int row = -1;
        int col = -1;
    };

    class CT_EXPORT CustomTree : public QTreeWidget
    {
        Q_OBJECT
    public:
        explicit CustomTree(QWidget* parent = nullptr);

        /**
         * @brief 设置点云视图
         */
        void setCloudView(CloudView* cloudview) { m_cloudview = cloudview; }

        /**
         * @brief 设置属性显示窗口
         */
        void setPropertiesTable(QTableWidget* table) { m_table = table; }

        /**
         * @brief 设置处理进度条
         */
        void setProgressBar(QProgressBar* progress_bar) { m_progress_bar = progress_bar; }

        /**
         * @brief 设置输出窗口
         */
        void setConsole(Console* console) { m_console = console; }

        /**
         * @brief 设置父类项目的图标
         */
        void setParentIcon(const QIcon& icon) { m_parent_icon = icon; }

        /**
         * @brief 设置子类项目的图标
         */
        void setChildIcon(const QIcon& icon) { m_child_icon = icon; }

    protected:

        /**
         * @brief 打印日志
         */
        void printI(const QString& message) { m_console->print(LOG_INFO, message); }
        void printW(const QString& message) { m_console->print(LOG_WARNING, message); }
        void printE(const QString& message) { m_console->print(LOG_ERROR, message); }

        /**
         * @brief 获取选中项目的索引
         */
        std::vector<Index> getSelectedIndexs();

        /**
         * @brief 获取勾选项目的索引
         */
        std::vector<Index> getCheckedIndexs();

        /**
         * @brief 获取点击项目的索引
         */
        std::vector<Index> getClickedIndexs(QTreeWidgetItem* item);

        /**
         * @brief 获取所有项目的索引
         */
        std::vector<Index> getAllIndexs();

        /**
         * @brief 返回索引是否有效
         */
        bool indexIsValid(const Index& index);

        /**
         * @brief 添加项目
         */
        void addItem(const Index& index, const QString& parent_id, const QString& child_id, bool selected = false);

        /**
         * @brief 移除项目
         */
        void removeItem(const Index& index);

        /**
         * @brief 返回该索引的项目
         */
        QTreeWidgetItem* item(const Index& index);

        /**
         * @brief 返回该项目名称的索引
         */
        Index index(const QString& text);

        /**
         * @brief 设置项目是否勾选
         */
        void setItemChecked(const Index& index, bool checked);

        /**
         * @brief 按照规则排序索引
         */
        std::vector<Index> getSortedIndexs(sort_type type, const std::vector<Index>& indexs);

    private slots:
        /**
         * @brief 项目选中改变事件
         */
        void itemSelectionChangedEvent();

        /**
         * @brief 项目点击事件
         */
        void itemClickedEvent(QTreeWidgetItem*, int i = 0);

    public:
        CloudView* m_cloudview;
        Console* m_console;
        QTableWidget* m_table;
        QProgressBar* m_progress_bar;
    private:
        QIcon m_parent_icon;
        QIcon m_child_icon;
    };

} // namespace pca

#endif
