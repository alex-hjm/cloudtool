/**
 * @file cloudtree.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CLOUDTREE_H
#define CT_BASE_CLOUDTREE_H

#include "base/customtree.h"
#include "base/fileio.h"

#include <QMenu>
#include <QInputDialog>
#include <QThread>

#define CLONE_ADD_FLAG  "clone-"
#define MERGE_ADD_FLAG  "merge-"

#define INIT_PATH       "../../../data"
#define SAMPLE_PATH     "../../../data/rabbit.pcd"

namespace ct
{
    class CT_EXPORT CloudTree : public CustomTree
    {
        Q_OBJECT
    public:
        explicit CloudTree(QWidget* parent = nullptr);

        ~CloudTree() override;

        /**
         * @brief 添加点云
         */
        void addCloud();

        /**
         * @brief 添加实例点云
         */
        void addSampleCloud() { this->loadPointCloud(SAMPLE_PATH); }

        /**
         * @brief 更新点云
         */
        void updateCloud(const Cloud::Ptr& cloud, const Cloud::Ptr& new_cloud, bool update_name = false);

        /**
         * @brief 追加点云到同一层
         */
        void appendCloud(const Cloud::Ptr& cloud, const Cloud::Ptr& new_cloud, bool selected = false)
        {
            new_cloud->update();
            Index index(index(cloud->id()).row, -1);
            insertCloud(index, new_cloud, selected);
        }

        /**
         * @brief 追加点云到新一层
         */
        void appendCloud(const Cloud::Ptr& new_cloud, bool selected = false)
        {
            insertCloud(Index(-1, -1), new_cloud, selected);
        }

        /**
         * @brief 获取选中的点云
         */
        std::vector<Cloud::Ptr> getSelectedClouds()
        {
            std::vector<Cloud::Ptr> clouds;
            for (auto& index : getSelectedIndexs())
                clouds.push_back(getCloud(index));
            return clouds;
        }

        /**
         * @brief 获取勾选的点云
         */
        std::vector<Cloud::Ptr> getCheckedClouds()
        {
            std::vector<Cloud::Ptr> clouds;
            for (auto& index : getCheckedIndexs())
                clouds.push_back(getCloud(index));
            return clouds;
        }

        /**
         * @brief 获取所有点云
         */
        std::vector<Cloud::Ptr> getAllClouds()
        {
            std::vector<Cloud::Ptr> clouds;
            for (auto& index : getAllIndexs())
                clouds.push_back(getCloud(index));
            return clouds;
        }

        /**
         * @brief 删除选中的点云
         */
        void removeSelectedClouds()
        {
            for (auto& index : getSortedIndexs(DESCENDING, getSelectedIndexs()))
                removeCloud(index);
        }

        /**
         * @brief 移除所有的点云
         */
        void removeAllClouds();

        /**
         * @brief 保存选中的点云
         */
        void saveSelectedClouds()
        {
            for (auto& index : getSelectedIndexs())
                saveCloud(index);
        }

        /**
         * @brief 保存所有的点云
         */
        void saveAllClouds()
        {
            for (auto& index : getAllIndexs())
                saveCloud(index);
        }

        /**
         * @brief 合并选中的点云项目
         */
        void mergeSelectedClouds();

        /**
         * @brief 重命名选中的点云项目
         */
        void renameSelectedClouds();

        /**
         * @brief 克隆选中的点云项目
         */
        void cloneSelectedClouds()
        {
            for (auto& index : getSelectedIndexs())
                cloneCloud(index);
        }

        /**
         * @brief 设置勾选点云
         */
        void setCloudChecked(const Cloud::Ptr& cloud, bool checked = true);

        /**
         * @brief 设置选中点云
         */
        void setCloudSelected(const Cloud::Ptr& cloud, bool selected = true);

        /**
         * @brief 显示进度条
         */
        void showProgressBar() { if(m_progress_bar) m_progress_bar->show(); }

        /**
         * @brief 关闭进度条
         */
        void closeProgressBar() { if(m_progress_bar) m_progress_bar->close(); }

    protected:
        /**
         * @brief 插入点云
         */
        void insertCloud(const Index& index, const Cloud::Ptr& cloud, bool selected = false);

        /**
         * @brief 移除该索引的点云
         */
        void removeCloud(const Index& index);

        /**
         * @brief 保存该索引的点云
         */
        void saveCloud(const Index& index);

        /**
         * @brief 克隆该索引的点云
         */
        void cloneCloud(const Index& index);

        /**
         * @brief 重命名该索引的点云
         */
        void renameCloud(const Index& index, const QString& name);

        /**
         * @brief 返回该索引的点云
         */
        Cloud::Ptr getCloud(const Index& index)
        {
            return m_cloud_vec[index.row][index.col];
        }

        /**
         * @brief 设置是否接受拖放文件
         */
        void setAcceptDrops(bool enable);

        /**
         * @brief 设置是否支持多选操作
         */
        void setExtendedSelectionMode(bool enable);

    signals:
        /**
         * @brief 加载点云文件
         */
        void loadPointCloud(const QString& filename);

        /**
         * @brief 保存点云文件
         */
        void savePointCloud(const Cloud::Ptr& cloud, const QString& filename, bool isBinary);

        /**
         * @brief 删除点云的ID
         */
        void removedCloudId(const QString&);

    private slots:

        /**
         * @brief 加载点云文件的结果
         */
        void loadCloudResult(bool success, const Cloud::Ptr& cloud, float time);

        /**
         * @brief 保存点云文件的结果
         */
        void saveCloudResult(bool success, const QString& path, float time);

        /**
         * @brief 项目点击事件
         */
        void itemClickedEvent(QTreeWidgetItem*, int);

        /**
         * @brief 项目选中改变事件
         */
        void itemSelectionChangedEvent();

    protected:
        void mousePressEvent(QMouseEvent* event) override;

    private:
        QString m_path;
        QThread m_thread;
        FileIO* m_fileio;
        QMenu* m_tree_menu;
        std::vector<std::vector<Cloud::Ptr>> m_cloud_vec;
    };
} // namespace ct

#endif // CT_BASE_CLOUDTREE_H