/**
 * @file cloudtree.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#include "base/cloudtree.h"

#include <QCheckBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QSpinBox>

#define CLONE_PREFIX    "clone-"
#define MERGE_PREFIX    "merge-"
#define INIT_PATH       "../../../data"

namespace ct
{
    CloudTree::CloudTree(QWidget* parent)
        : CustomTree(parent),
        m_path(INIT_PATH),
        m_thread(this),
        m_tree_menu(nullptr),
        m_progress_bar(nullptr),
        m_table(nullptr),
        m_console(nullptr),
        m_cloudview(nullptr)
    {
        // register meta type
        qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr &");
        qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr");

        // move to thread
        m_fileio = new FileIO;
        m_fileio->moveToThread(&m_thread);
        connect(&m_thread, &QThread::finished, m_fileio, &QObject::deleteLater);
        connect(m_fileio, &FileIO::loadCloudResult, this, &CloudTree::loadCloudResult);
        connect(m_fileio, &FileIO::saveCloudResult, this, &CloudTree::saveCloudResult);
        connect(this, &CloudTree::loadPointCloud, m_fileio, &FileIO::loadPointCloud);
        connect(this, &CloudTree::savePointCloud, m_fileio, &FileIO::savePointCloud);
        m_thread.start();

        connect(this, &CloudTree::itemClicked, this, &CloudTree::itemClickedEvent);
        connect(this, &CloudTree::itemSelectionChanged, this, &CloudTree::itemSelectionChangedEvent);
    }

    CloudTree::~CloudTree()
    {
        m_thread.quit();
        m_thread.wait();
    }

    void CloudTree::addCloud()
    {
        QString filter = "all(*.*);;ply(*.ply);;pcd(*.pcd)";
        QStringList filePathList = QFileDialog::getOpenFileNames(this, tr("Open pointcloud file"), m_path, filter);
        if (filePathList.isEmpty())
            return;
        if (m_progress_bar != nullptr)
            m_progress_bar->show();
        for (auto& i : filePathList)
            emit loadPointCloud(i);
    }

    void CloudTree::insertCloud(const Index& index, const Cloud::Ptr& cloud, bool selected)
    {
        // check cloud id
        if (cloud == nullptr)
            return;
        if (!cloud->empty() && m_cloudview->contains(cloud->id()))
        {
            int k = QMessageBox::warning(this, "WARNING", "Rename the exists id?", QMessageBox::Yes, QMessageBox::Cancel);
            if (k == QMessageBox::Yes)
            {
                bool ok = false;
                QString res = QInputDialog::getText(this, "Rename", "", QLineEdit::Normal, cloud->id(), &ok);
                if (ok)
                {
                    if (res == cloud->id() || m_cloudview->contains(res))
                    {
                        m_console->print(LOG_ERROR, "The id " + res + " already exists!");
                        return;
                    }
                    cloud->setId(res);
                }
                else
                {
                    m_console->print(LOG_ERROR, "Add pointcloud canceled.");
                    return;
                }
            }
        }

        // update cloud_vec
        if (index.row <= -1 || index.row > topLevelItemCount())
        {
            std::vector<Cloud::Ptr> temp;
            temp.push_back(cloud);
            m_cloud_vec.push_back(temp);
        }
        else
        {
            if ((index.col > -1) && (index.col < topLevelItem(index.row)->childCount()))
                m_cloud_vec[index.row].insert(m_cloud_vec[index.row].begin() + index.col, cloud);
            else
                m_cloud_vec[index.row].push_back(cloud);
        }
        // add treewidget item
        addItem(index, cloud->info().absolutePath(), cloud->id(), selected);
        m_cloudview->addPointCloud(cloud);
        m_cloudview->resetCamera();
    }

    void CloudTree::updateCloud(const Cloud::Ptr& cloud, const Cloud::Ptr& new_cloud, bool update_name)
    {
        if (cloud == nullptr || new_cloud == nullptr) return;
        cloud->swap(*new_cloud);
        cloud->update();
        Index i = index(cloud->id());
        if (update_name) renameCloud(i, new_cloud->id());
        m_cloudview->addPointCloud(cloud);
        if (item(i)->isSelected()) m_cloudview->addBox(cloud);
    }

    void CloudTree::removeAllClouds()
    {
        for (auto i : m_cloud_vec)
            for (auto j : i)
                emit removedCloudId(j->id());
        m_cloudview->removeAllPointClouds();
        m_cloudview->removeAllShapes();
        this->clear();
        this->m_cloud_vec.clear();
        std::vector<std::vector<Cloud::Ptr>>().swap(m_cloud_vec);
    }

    void CloudTree::mergeSelectedClouds()
    {
        std::vector<Cloud::Ptr> clouds = getSelectedClouds();
        if (clouds.size() <= 1)
        {
            m_console->print(LOG_WARNING, "The number of pointclouds to merge are not enough!");
            return;
        }
        Cloud::Ptr merged_cloud(new Cloud);
        for (auto& i : clouds)
            *merged_cloud += *i;
        merged_cloud->setId(MERGE_PREFIX + clouds.front()->id());
        merged_cloud->setInfo(clouds.front()->info());
        merged_cloud->update();
        appendCloud(merged_cloud);
    }

    void CloudTree::renameSelectedClouds()
    {
        for (auto& index : getSelectedIndexs())
        {
            Cloud::Ptr cloud = getCloud(index);
            bool ok = false;
            QString name = QInputDialog::getText(this, "Rename", "", QLineEdit::Normal, cloud->id(), &ok);
            if (ok)
                renameCloud(index, name);
            else
            {
                m_console->print(LOG_WARNING, "Rename pointcloud canceled.");
                return;
            }
        }
    }

    void CloudTree::setCloudChecked(const Cloud::Ptr& cloud, bool checked)
    {
        if (cloud == nullptr)
            return;
        Index i = index(cloud->id());
        if (checked && (item(i)->checkState(0) == Qt::Checked))
            return;
        if ((!checked) && (item(i)->checkState(0) == Qt::Unchecked))
            return;
        this->setItemChecked(i, checked);
        if (checked)
        {
            m_cloudview->addPointCloud(cloud);
            m_cloudview->addBox(cloud);
        }
        else
        {
            m_cloudview->removePointCloud(cloud->id());
            m_cloudview->removeShape(cloud->boxId());
            m_cloudview->removePointCloud(cloud->normalId());
        }
    }

    void CloudTree::setCloudSelected(const Cloud::Ptr& cloud, bool selected)
    {
        if (cloud == nullptr)
            return;
        Index i = index(cloud->id());
        item(i)->setSelected(selected);
    }

    void CloudTree::removeCloud(const Index& index)
    {
        Cloud::Ptr cloud = getCloud(index);
        emit removedCloudId(cloud->id());
        m_cloudview->removePointCloud(cloud->id());
        m_cloudview->removeShape(cloud->boxId());
        m_cloudview->removePointCloud(cloud->normalId());
        removeItem(index);
        if (m_cloud_vec[index.row].size() == 1)
            m_cloud_vec.erase(m_cloud_vec.begin() + index.row);
        else
            m_cloud_vec[index.row].erase(m_cloud_vec[index.row].begin() + index.col);
    }

    void CloudTree::saveCloud(const Index& index)
    {
        Cloud::Ptr cloud = getCloud(index);
        QString filter = "ply(*.ply);;pcd(*.pcd)";
        QString filepath = QFileDialog::getSaveFileName(this, tr("Save pointcloud"), cloud->id(), filter);
        if (filepath.isEmpty())
            return;
        QMessageBox message_box(QMessageBox::NoIcon, "Saved format", tr("Save in binary or ascii format?"),
                                QMessageBox::NoButton, this);
        message_box.addButton(tr("Ascii"), QMessageBox::ActionRole);
        message_box.addButton(tr("Binary"), QMessageBox::ActionRole)->setDefault(true);
        message_box.addButton(QMessageBox::Cancel);
        int k = message_box.exec();
        if (k == QMessageBox::Cancel)
        {
            m_console->print(LOG_WARNING, "Save pointcloud canceled.");
            return;
        }
        if (m_progress_bar != nullptr)
            m_progress_bar->show();
        emit savePointCloud(cloud, filepath, k);
    }

    void CloudTree::cloneCloud(const Index& index)
    {
        Cloud::Ptr clone_cloud = getCloud(index)->makeShared();
        clone_cloud->setId(CLONE_PREFIX + clone_cloud->id());
        appendCloud(clone_cloud);
    }

    void CloudTree::renameCloud(const Index& index, const QString& name)
    {
        Cloud::Ptr cloud = getCloud(index);
        if (m_cloudview->contains(name))
        {
            m_console->print(LOG_WARNING, "The id " + name + " already exists!");
            return;
        }
        item(index)->setText(0, name);
        m_cloudview->removePointCloud(cloud->id());
        m_cloudview->removePointCloud(cloud->normalId());
        m_cloudview->removeShape(cloud->boxId());
        cloud->setId(name);
    }

    void CloudTree::setAcceptDrops(bool enable)
    {
        if (enable)
        {
            m_cloudview->setAcceptDrops(true);
            connect(m_cloudview, &CloudView::dropFilePath, [=](const QStringList& filepath)
                    {for (auto& i : filepath) emit loadPointCloud(i); });
        }
        else
        {
            m_cloudview->setAcceptDrops(false);
            disconnect(m_cloudview, &CloudView::dropFilePath, this, 0);
        }
    }

    void CloudTree::setExtendedSelectionMode(bool enable)
    {
        if (enable)
            this->setSelectionMode(QAbstractItemView::ExtendedSelection);
        else
            this->setSelectionMode(QAbstractItemView::SingleSelection);
    }

    void CloudTree::loadCloudResult(bool success, const Cloud::Ptr& cloud, float time)
    {
        if (!success)
            m_console->print(LOG_ERROR, "Failed to save the file!");
        else
        {
            m_console->print(LOG_INFO, LOG_STATU_PROCESS_DONE("Load the file [path: " + cloud->info().absoluteFilePath() + " ]", time));
            m_path = cloud->info().path();
            appendCloud(cloud);
        }
        if (m_progress_bar != nullptr)
            m_progress_bar->close();
    }

    void CloudTree::saveCloudResult(bool success, const QString& path, float time)
    {
        if (!success)
            m_console->print(LOG_ERROR, "Save the file failed !");
        else
        {
            m_console->print(LOG_INFO, LOG_STATU_PROCESS_DONE("Save the file [path: " + path + " ]", time));
        }
        if (m_progress_bar != nullptr)
            m_progress_bar->close();
    }

    void CloudTree::itemClickedEvent(QTreeWidgetItem* item, int)
    {
        std::vector<Index> index = getClickedIndexs(item);
        for (auto& i : index)
        {
            Cloud::Ptr cloud = getCloud(i);
            if (item->checkState(0) == Qt::Checked)
            {
                m_cloudview->addPointCloud(cloud);
                m_cloudview->addBox(cloud);
            }
            else
            {
                m_cloudview->removePointCloud(cloud->id());
                m_cloudview->removeShape(cloud->boxId());
                m_cloudview->removePointCloud(cloud->normalId());
            }
        }
    }

    void CloudTree::itemSelectionChangedEvent()
    {
        // update box
        std::vector<Index> all = getAllIndexs();
        std::vector<Index> indexs = getSelectedIndexs();
        for (auto& i : all)
        {
            std::vector<Index>::const_iterator it = std::find(indexs.begin(), indexs.end(), i);
            Cloud::Ptr cloud = getCloud(i);
            if (it != indexs.end())
                m_cloudview->addBox(cloud);
            else
                m_cloudview->removeShape(cloud->boxId());
        }
        // update table
        if (m_table == nullptr)
            return;
        QString id, type, format, size, resolution;
        if (indexs.size() == 0)
        {
            id = type = size = resolution = "";
            m_table->removeCellWidget(4, 1); // point_size
            m_table->removeCellWidget(5, 1); // opacity
            m_table->removeCellWidget(6, 1); // normals
            m_cloudview->showCloudId("");
        }
        else
        {
            Cloud::Ptr update_cloud = getCloud(indexs.front());
            id = update_cloud->id();
            type = update_cloud->type();
            resolution = QString::number(update_cloud->resolution());
            size = QString::number(update_cloud->size());
            m_cloudview->showCloudId(update_cloud->id());

            // point_size
            QSpinBox* point_size = new QSpinBox;
            point_size->setRange(1, 99);
            point_size->setValue(update_cloud->pointSize());
            connect(point_size, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=](int value)
                    {
                        update_cloud->setPointSize(value);
                        m_cloudview->setPointCloudSize(update_cloud->id(), value);
                    });

            // opacity
            QDoubleSpinBox* opacity = new QDoubleSpinBox;
            opacity->setSingleStep(0.1);
            opacity->setRange(0, 1);
            opacity->setValue(update_cloud->opacity());
            connect(opacity, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
                    {
                        update_cloud->setOpacity(value);
                        m_cloudview->setPointCloudOpacity(update_cloud->id(), value);
                    });

            // has normals
            QCheckBox* show_normals = new QCheckBox;
            QDoubleSpinBox* scale = new QDoubleSpinBox;
            scale->setSingleStep(0.01);
            scale->setRange(0, 9999);
            scale->setValue(0.01);
            if (update_cloud->hasNormals())
                show_normals->setEnabled(true);
            else
                show_normals->setEnabled(false);
            connect(scale, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
                    {
                        if (update_cloud->hasNormals() && show_normals->isChecked())
                            m_cloudview->addPointCloudNormals(update_cloud, 1, value);
                    });
            connect(show_normals, &QCheckBox::stateChanged, [=](int state)
                    {
                        if (state) m_cloudview->addPointCloudNormals(update_cloud, 1, scale->value());
                        else m_cloudview->removeShape(update_cloud->normalId());
                    });

            QHBoxLayout* layout = new QHBoxLayout;
            layout->addWidget(show_normals);
            layout->addWidget(scale);
            layout->addStretch();
            QWidget* normals = new QWidget;
            normals->setLayout(layout);
            normals->layout()->setMargin(0);
            m_table->setCellWidget(4, 1, point_size);
            m_table->setCellWidget(5, 1, opacity);
            m_table->setCellWidget(6, 1, normals);
        }
        m_table->setItem(0, 1, new QTableWidgetItem(id));
        m_table->setItem(1, 1, new QTableWidgetItem(type));
        m_table->setItem(2, 1, new QTableWidgetItem(size));
        m_table->setItem(3, 1, new QTableWidgetItem(resolution));
    }

    void CloudTree::mousePressEvent(QMouseEvent* event)
    {
        QModelIndex indexSelect = indexAt(event->pos());
        if (event->button() == Qt::RightButton && indexSelect.row() != -1)
        {
            QTreeWidgetItem* item = this->itemFromIndex(indexSelect);
            if (item->isSelected())
            {
                if (m_tree_menu == nullptr)
                {
                    QMenu menu(this);
                    menu.addAction("clear", [=] { removeSelectedClouds(); });
                    menu.addAction("save", [=] { saveSelectedClouds(); });
                    if (item->checkState(0) == Qt::Checked)
                        menu.addAction("hide", [=] { for (auto& i : getSelectedClouds()) setCloudChecked(i, false); });
                    else if (item->checkState(0) == Qt::Unchecked)
                        menu.addAction("show", [=] { for (auto& i : getSelectedClouds()) setCloudChecked(i, true); });
                    menu.addAction("clone", [=] { cloneSelectedClouds(); });
                    menu.addAction("rename", [=] { renameSelectedClouds(); });
                    menu.exec(event->globalPos());
                }
                else
                    m_tree_menu->exec(event->globalPos());
                event->accept();
            }
        }
        return CustomTree::mousePressEvent(event);
    }

} // namespace ct
