/**
 * @file color.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-12
 */
#include "color.h"

#include "ui_color.h"

#define COLOR_POINTCLOUD    (0)
#define COLOR_BACKGROUNG    (1)
#define COLOR_NORMALS       (2)
#define COLOR_BOUNDINGBOX   (3)

Color::Color(QWidget* parent)
    : CustomDock(parent), ui(new Ui::Color), m_field(""), m_rgb(QColorConstants::White)
{
    ui->setupUi(this);
    ui->gridLayout->setSpacing(0);
    for (int row = 0; row < 5; row++)
    {
        for (int column = 0; column < 10; column++)
        {
            QPushButton* btn = new QPushButton();
            btn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            btn->setFixedHeight(20);
            if (row == 4 && column == 9)
            {
                btn->setText("+");
                btn->setStyleSheet(tr("QPushButton{border:none;border-radius:4px;background-color:transparent;}"
                                      "QPushButton:pressed{background-color:lightgray;}"));
                connect(btn, &QPushButton::clicked, [=]
                        {
                            QColor color = QColorDialog::getColor(Qt::white, this, tr("select color"));
                            emit rgb(QColor(color.red(), color.green(), color.blue()));
                        });
            }
            else
            {
                btn->setStyleSheet(tr("QPushButton{border:none;border-radius:4px;background-color:rgb(%1, %2, %3);}"
                                      "QPushButton:pressed{background-color:lightgray;}")
                                   .arg((colors[row][column]).red())
                                   .arg((colors[row][column]).green())
                                   .arg((colors[row][column]).blue()));
                connect(btn, &QPushButton::clicked, [=]
                        {
                            QColor color = colors[row][column];
                            emit rgb(QColor(color.red(), color.green(), color.blue()));
                        });
            }
            ui->gridLayout->addWidget(btn, row, column);
        }
    }
    connect(ui->btn_x, &QPushButton::clicked, [=] { emit field("x"); });
    connect(ui->btn_y, &QPushButton::clicked, [=] { emit field("y"); });
    connect(ui->btn_z, &QPushButton::clicked, [=] { emit field("z"); });
    connect(ui->btn_apply, &QPushButton::clicked, this, &Color::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Color::reset);
    connect(this, SIGNAL(rgb(const QColor&)), this, SLOT(setColor(const QColor&)));
    connect(this, SIGNAL(field(const QString&)), this, SLOT(setColor(const QString&)));
}

Color::~Color() { delete ui; }

void Color::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    switch (ui->cbox_type->currentIndex())
    {
    case COLOR_POINTCLOUD:
        for (auto& cloud : selected_clouds)
        {
            if (m_field != "") cloud->setCloudColor(m_field);
            else cloud->setCloudColor(m_rgb);
            m_cloudview->addPointCloud(cloud);
        }
        break;
    case COLOR_BACKGROUNG:
        break;
    case COLOR_NORMALS:
        for (auto& cloud : selected_clouds)
            cloud->setNormalColor(m_rgb);
        break;
    case 3: // box
        for (auto& cloud : selected_clouds)
            cloud->setBoxColor(m_rgb);
        break;
    }
}

void Color::reset()
{
    m_rgb == QColorConstants::White, m_field = "";
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    switch (ui->cbox_type->currentIndex())
    {
    case COLOR_POINTCLOUD:
        for (auto& cloud : selected_clouds)
            m_cloudview->resetPointCloudColor(cloud);
        break;
    case COLOR_BACKGROUNG:
        m_cloudview->resetBackgroundColor();
        break;
    case COLOR_NORMALS:
        for (auto& cloud : selected_clouds)
            if (m_cloudview->contains(cloud->normalId()))
                m_cloudview->setPointCloudColor(cloud->normalId(), QColorConstants::White);
        break;
    case COLOR_BOUNDINGBOX:
        for (auto& cloud : selected_clouds)
            if (m_cloudview->contains(cloud->boxId()))
                m_cloudview->setShapeColor(cloud->boxId(), QColorConstants::White);
        break;
    }
}

void Color::setColor(const QColor& rgb)
{
    m_rgb = rgb, m_field = "";
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    switch (ui->cbox_type->currentIndex())
    {
    case COLOR_POINTCLOUD:
        for (auto& cloud : selected_clouds)
            m_cloudview->setPointCloudColor(cloud, rgb);
        break;
    case COLOR_BACKGROUNG:
        m_cloudview->setBackgroundColor(rgb);
        break;
    case COLOR_NORMALS:
        for (auto& cloud : selected_clouds)
            if (m_cloudview->contains(cloud->normalId()))
                m_cloudview->setPointCloudColor(cloud->normalId(), rgb);
        break;
    case COLOR_BOUNDINGBOX:
        for (auto& cloud : selected_clouds)
            if (m_cloudview->contains(cloud->boxId()))
                m_cloudview->setShapeColor(cloud->boxId(), rgb);
        break;
    }
}

void Color::setColor(const QString& field)
{
    m_field = field, m_rgb = QColorConstants::White;
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    for (auto& i : selected_clouds) m_cloudview->setPointCloudColor(i, field);
}
