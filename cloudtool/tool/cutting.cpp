/**
 * @file cutting.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "cutting.h"
#include "ui_cutting.h"

#define CUT_TYPE_RECTANGULAR    0
#define CUT_TYPE_POLYGONAL      1

#define POLYGONAL_ID            "polygonal"

#define CUTTING_PRE_FLAG        "-cutting"
#define CUTTING_ADD_FLAG        "cutting-"

Cutting::Cutting(QWidget* parent) :CustomDialog(parent),
ui(new Ui::Cutting), is_picking(false), pick_start(false)
{
    ui->setupUi(this);

    connect(ui->btn_add, &QPushButton::clicked, this, &Cutting::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Cutting::apply);
    connect(ui->btn_start, &QPushButton::clicked, this, &Cutting::start);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Cutting::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &Cutting::close);
    connect(ui->btn_selectin, &QPushButton::clicked, [=] {this->cuttingCloud(false);});
    connect(ui->btn_selectout, &QPushButton::clicked, [=] {this->cuttingCloud(true);});
    ui->cbox_type->setCurrentIndex(0);
}

Cutting::~Cutting()
{
    delete ui;
}

void Cutting::init()
{
    connect(ui->cbox_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Cutting::updateInfo);
    this->updateInfo(ui->cbox_type->currentIndex());
}

void Cutting::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_cutting_map.find(cloud->id()) == m_cutting_map.end())
        {
            printW(QString("The cloud[id:%1] has no cutted cloud !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_cutting_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        m_cloudview->removeShape(new_cloud->boxId());
        new_cloud->setId(CUTTING_ADD_FLAG + cloud->id());
        m_cloudtree->appendCloud(cloud, new_cloud, true);
        m_cutting_map.erase(cloud->id());
        printI(QString("Add cutted cloud[id:%1] done.").arg(new_cloud->id()));
    }
    m_pick_points.clear();
}

void Cutting::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_cutting_map.find(cloud->id()) == m_cutting_map.end())
        {
            printW(QString("The cloud[id:%1] has no cutted cloud !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_cutting_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        m_cloudview->removeShape(new_cloud->boxId());
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_cutting_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply cutted cloud[id:%1] done.").arg(new_cloud->id()));
    }
    m_pick_points.clear();
}

void Cutting::start()
{
    if (!is_picking)
    {
        std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
        if (selected_clouds.empty())
        {
            printW("Please select a cloud!");
            return;
        }

        is_picking = true;
        m_cloudtree->setEnabled(false);
        m_cloudview->setInteractorEnable(false);

        for (auto& cloud : selected_clouds)
            m_cloudview->removeShape(cloud->boxId());
        for (auto& cut_cloud : m_cutting_map)
        {
            m_cloudview->removePointCloud(cut_cloud.second->id());
            m_cloudview->removeShape(cut_cloud.second->boxId());
        }
        m_cloudview->removeShape(POLYGONAL_ID);

        ui->btn_start->setIcon(QIcon(":/res/icon/stop.svg"));
        connect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &Cutting::mouseLeftPressed);
        connect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &Cutting::mouseLeftReleased);
        connect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &Cutting::mouseRightReleased);
        connect(m_cloudview, &ct::CloudView::mouseMoved, this, &Cutting::mouseMoved);
        this->updateInfo(ui->cbox_type->currentIndex());
    }
    else
    {
        is_picking = false;
        m_cloudtree->setEnabled(true);
        disconnect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &Cutting::mouseLeftPressed);
        disconnect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &Cutting::mouseLeftReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &Cutting::mouseRightReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseMoved, this, &Cutting::mouseMoved);
        ui->btn_start->setIcon(QIcon(":/res/icon/start.svg"));
        this->updateInfo(ui->cbox_type->currentIndex());
    }
}

void Cutting::reset()
{
    if (is_picking) this->start();
    pick_start = false;
    m_cloudview->setInteractorEnable(true);
    m_cloudview->removeShape(POLYGONAL_ID);
    for (auto& cut_cloud : m_cutting_map)
    {
        m_cloudview->removePointCloud(cut_cloud.second->id());
        m_cloudview->removeShape(cut_cloud.second->boxId());
    }
    m_pick_points.clear();
}

void Cutting::updateInfo(int index)
{
    if (index == CUT_TYPE_RECTANGULAR)
    {
        if (is_picking)
            m_cloudview->showInfo("Segmentation [ON] (rectangular selection)", 1);
        else
            m_cloudview->showInfo("Segmentation [OFF] (rectangular selection)", 1);
        m_cloudview->showInfo("Left/Right click : set opposite corners", 2);
    }
    else if (index == CUT_TYPE_POLYGONAL)
    {
        if (is_picking)
            m_cloudview->showInfo("Segmentation [ON] (polygonal selection)", 1);
        else
            m_cloudview->showInfo("Segmentation [OFF] (polygonal selection)", 1);
        m_cloudview->showInfo("Left click : add contour points / Right click : close", 2);
    }
}

void Cutting::cuttingCloud(bool select_in)
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (m_pick_points.size() < 3)
    {
        printW("Picked points are not enough !");
        return;
    }
    if (is_picking) this->start();
    m_cloudview->setInteractorEnable(true);
    m_cloudview->removeShape(POLYGONAL_ID);
    for (auto& cloud : selected_clouds)
    {
        std::vector<int> indices = m_cloudview->areaPick(m_pick_points, cloud, select_in);
        ct::Cloud::Ptr cut_cloud = cloud->makeShared();
        pcl::copyPointCloud(*cloud, indices, *cut_cloud);
        cut_cloud->setId(cloud->id() + CUTTING_PRE_FLAG);
        cut_cloud->update();
        m_cloudview->addPointCloud(cut_cloud);
        m_cloudview->addBox(cut_cloud);
        m_cloudview->setPointCloudColor(cut_cloud->id(), QColorConstants::Green);
        m_cloudview->setPointCloudSize(cut_cloud->id(), cut_cloud->pointSize() + 2);
        m_cutting_map[cloud->id()] = cut_cloud;
    }
}

void Cutting::mouseLeftPressed(const ct::PointXY& pt)
{
    if (!pick_start)
    {
        m_pick_points.clear();
        m_pick_points.push_back(pt);
        pick_start = true;
    }
}

void Cutting::mouseLeftReleased(const ct::PointXY& pt)
{
    if (pick_start)
    {
        if ((m_pick_points.front().x == pt.x) && (m_pick_points.front().y == pt.y))
            return;
        else
        {
            if (ui->cbox_type->currentIndex() == CUT_TYPE_RECTANGULAR)
            {
                ct::PointXY p1(m_pick_points.front().x, pt.y);
                ct::PointXY p2(pt.x, m_pick_points.front().y);
                m_pick_points.push_back(p1);
                m_pick_points.push_back(pt);
                m_pick_points.push_back(p2);
                m_cloudview->addPolygon2D(m_pick_points, POLYGONAL_ID, QColorConstants::Green);
                pick_start = false;
            }
            else
            {
                m_pick_points.push_back(pt);
            }
        }
    }
}

void Cutting::mouseRightReleased(const ct::PointXY& pt)
{
    if (pick_start)
    {
        if (ui->cbox_type->currentIndex() == CUT_TYPE_RECTANGULAR)
        {
            ct::PointXY p1(m_pick_points.front().x, pt.y);
            ct::PointXY p2(pt.x, m_pick_points.front().y);
            m_pick_points.push_back(p1);
            m_pick_points.push_back(pt);
            m_pick_points.push_back(p2);
            pick_start = false;
        }
        else
        {
            if (m_pick_points.size() == 2)  m_pick_points.push_back(pt);
            m_cloudview->addPolygon2D(m_pick_points, POLYGONAL_ID, QColorConstants::Green);
            pick_start = false;
        }
    }

}

void Cutting::mouseMoved(const ct::PointXY& pt)
{
    if (pick_start)
    {
        if (ui->cbox_type->currentIndex() == CUT_TYPE_RECTANGULAR)
        {
            std::vector<ct::PointXY> pre_points = m_pick_points;
            ct::PointXY p1(pre_points.front().x, pt.y);
            ct::PointXY p2(pt.x, pre_points.front().y);
            pre_points.push_back(p1);
            pre_points.push_back(pt);
            pre_points.push_back(p2);
            m_cloudview->addPolygon2D(pre_points, POLYGONAL_ID, QColorConstants::Green);
        }
        else
        {
            std::vector<ct::PointXY> pre_points = m_pick_points;
            pre_points.push_back(pt);
            m_cloudview->addPolygon2D(pre_points, POLYGONAL_ID, QColorConstants::Green);
        }
    }
}
