/**
 * @file pickpoints.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "pickpoints.h"
#include "ui_pickpoints.h"

#define PICK_TYPE_TWOPOINTS     0
#define PICK_TYPE_MULTPOINTS    1

#define POLYGONAL_ID            "polygonal"
#define ARROW_ID                "arrow"
#define PICKING_PRE_FLAG        "-picking"
#define PICKING_ADD_FLAG        "picking-"

PickPoints::PickPoints(QWidget* parent) :
    CustomDialog(parent), ui(new Ui::PickPoints),
    is_picking(false),
    pick_start(false),
    m_pick_cloud(new ct::Cloud),
    m_pick_point(-1, -1)
{
    ui->setupUi(this);
    connect(ui->btn_start, &QPushButton::clicked, this, &PickPoints::start);
    connect(ui->btn_reset, &QPushButton::clicked, this, &PickPoints::reset);
    connect(ui->btn_add, &QPushButton::clicked, this, &PickPoints::add);
    connect(ui->btn_close, &QPushButton::clicked, this, &PickPoints::close);
    ui->cbox_type->setCurrentIndex(0);
}

PickPoints::~PickPoints()
{
    delete ui;
}

void PickPoints::init()
{
    connect(ui->cbox_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &PickPoints::updateInfo);
    this->updateInfo(ui->cbox_type->currentIndex());
}

void PickPoints::start()
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
        m_selected_cloud = selected_clouds.front();
        m_cloudview->removeShape(m_selected_cloud->boxId());
        for (auto& c : m_cloudtree->getAllClouds())
            if (c->id() != m_selected_cloud->id())
            {
                m_cloudtree->setCloudSelected(c, false);
                m_cloudtree->setCloudChecked(c, false);
            }
        m_cloudview->removeShape(POLYGONAL_ID);

        ui->btn_start->setIcon(QIcon(":/res/icon/stop.svg"));
        connect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &PickPoints::mouseLeftPressed);
        connect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &PickPoints::mouseLeftReleased);
        connect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &PickPoints::mouseRightReleased);
        connect(m_cloudview, &ct::CloudView::mouseMoved, this, &PickPoints::mouseMoved);
        this->updateInfo(ui->cbox_type->currentIndex());
    }
    else
    {
        is_picking = false;
        disconnect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &PickPoints::mouseLeftPressed);
        disconnect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &PickPoints::mouseLeftReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &PickPoints::mouseRightReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseMoved, this, &PickPoints::mouseMoved);
        ui->btn_start->setIcon(QIcon(":/res/icon/start.svg"));
        m_cloudview->removeShape(ARROW_ID);
        m_cloudview->removeShape(POLYGONAL_ID);
        m_cloudview->removePointCloud(m_pick_cloud->id());
        m_cloudview->clearInfo();
        this->updateInfo(ui->cbox_type->currentIndex());
    }
}

void PickPoints::add()
{
    if (m_selected_cloud == nullptr)
    {
        printW("Please select a cloud!");
        return;
    }
    if (m_pick_cloud->empty())
    {
        printW("Please pick points first!");
        return;
    }
    ct::Cloud::Ptr new_cloud = m_pick_cloud->makeShared();
    m_cloudview->removePointCloud(new_cloud->id());
    new_cloud->setId(PICKING_ADD_FLAG + m_selected_cloud->id());
    m_cloudtree->appendCloud(m_selected_cloud, new_cloud, true);
    this->reset();
    printI(QString("Add picking cloud[id:%1] done.").arg(new_cloud->id()));
}

void PickPoints::reset()
{
    if (is_picking) this->start();
    pick_start = false;
    m_cloudview->removeShape(POLYGONAL_ID);
    m_cloudview->removeShape(ARROW_ID);
    m_pick_cloud->clear();
    m_selected_cloud = nullptr;
}

void PickPoints::updateInfo(int index)
{
    if (index == PICK_TYPE_TWOPOINTS)
    {
        if (is_picking)
            m_cloudview->showInfo("PickPoints [ON] (two points picking)", 1);
        else
            m_cloudview->showInfo("PickPoints [OFF] (two points picking)", 1);
        m_cloudview->showInfo("Left click : pick point", 2);
    }
    else if (index == PICK_TYPE_MULTPOINTS)
    {
        if (is_picking)
            m_cloudview->showInfo("PickPoints [ON] (multil points picking)", 1);
        else
            m_cloudview->showInfo("PickPoints [OFF] (multil points picking)", 1);
        m_cloudview->showInfo("Left click : add points / Right click : close", 2);
    }
}

void PickPoints::mouseLeftPressed(const ct::PointXY& pt)
{
    if (!pick_start)
    {
        int start = m_cloudview->singlePick(pt);
        if (start == -1 || start == 1) return;
        m_pick_point = pt;
        m_pick_cloud->clear();
        ct::PointXYZRGBN start_pt = m_selected_cloud->points[start];
        m_pick_cloud->push_back(start_pt);
        m_pick_cloud->setId(m_selected_cloud->id() + PICKING_PRE_FLAG);
        m_cloudview->removeShape(ARROW_ID);
        m_cloudview->addPointCloud(m_pick_cloud);
        m_cloudview->setPointCloudColor(m_pick_cloud, ct::Color::Red);
        m_cloudview->setPointCloudSize(m_pick_cloud->id(), m_pick_cloud->pointSize() + 3);
        pick_start = true;
        if (ui->cbox_type->currentIndex() == PICK_TYPE_TWOPOINTS)
            m_cloudview->showInfo(tr("Start Point[%1]: %2, %3, %4").arg(start).arg(start_pt.x, 0, 'g', 3).
                                  arg(start_pt.y, 0, 'g', 3).arg(start_pt.z, 0, 'g', 3), 3, ct::Color::Yellow);
        else
            m_cloudview->showInfo(tr("Pick Point[%1]: %2, %3, %4").arg(start).arg(start_pt.x, 0, 'g', 3).
                                  arg(start_pt.y, 0, 'g', 3).arg(start_pt.z, 0, 'g', 3), 3, ct::Color::Yellow);
    }
}

void PickPoints::mouseLeftReleased(const ct::PointXY& pt)
{
    if (pick_start)
    {
        if ((m_pick_point.x == pt.x) && (m_pick_point.y == pt.y))
            return;
        else
        {
            int end = m_cloudview->singlePick(pt);
            if (end == -1 || end == 1) return;
            ct::PointXYZRGBN end_pt = m_selected_cloud->points[end];
            m_pick_cloud->push_back(end_pt);
            m_cloudview->removeShape(POLYGONAL_ID);
            m_cloudview->addPointCloud(m_pick_cloud);
            m_cloudview->setPointCloudColor(m_pick_cloud, ct::Color::Red);
            m_cloudview->setPointCloudSize(m_pick_cloud->id(), m_pick_cloud->pointSize() + 3);
            if (ui->cbox_type->currentIndex() == PICK_TYPE_TWOPOINTS)
            {
                ct::PointXYZRGBN start_pt = m_pick_cloud->points.front();
                m_cloudview->addArrow(end_pt, start_pt, ARROW_ID, true, ct::Color::Green);
                float x = start_pt.x - end_pt.x;
                float y = start_pt.y - end_pt.y;
                float z = start_pt.z - end_pt.z;
                float distance = sqrt(x * x + y * y + z * z);
                m_cloudview->showInfo(tr("End Point[%1]: %2, %3, %4").arg(end).arg(end_pt.x, 0, 'g', 3).
                                      arg(end_pt.y, 0, 'g', 3).arg(end_pt.z, 0, 'g', 3), 4, ct::Color::Yellow);
                m_cloudview->showInfo(tr("Distance: %1").arg(distance, 0, 'g', 3), 5, ct::Color::Yellow);
                pick_start = false;
            }
        }
    }
}

void PickPoints::mouseRightReleased(const ct::PointXY& pt)
{
    if (pick_start)
    {
        if (ui->cbox_type->currentIndex() == PICK_TYPE_MULTPOINTS)
        {
            m_cloudview->removeShape(POLYGONAL_ID);
            m_cloudview->addPointCloud(m_pick_cloud);
            m_cloudview->setPointCloudColor(m_pick_cloud, ct::Color::Red);
            m_cloudview->setPointCloudSize(m_pick_cloud->id(), m_pick_cloud->pointSize() + 3);
            m_cloudview->addPolygon(m_pick_cloud, POLYGONAL_ID, ct::Color::Green);
            pick_start = false;
        }
    }
}

void PickPoints::mouseMoved(const ct::PointXY& pt)
{
    if (pick_start)
    {
        int tmp = m_cloudview->singlePick(pt);
        if (tmp == -1 || tmp == 1) return;
        ct::Cloud::Ptr tmp_cloud = m_pick_cloud->makeShared();
        tmp_cloud->push_back(m_selected_cloud->points[tmp]);
        m_cloudview->addPolygon(tmp_cloud, POLYGONAL_ID, ct::Color::Green);
    }
}

