/**
 * @file color.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-12
 */
#include "color.h"
#include "ui_color.h"

#include <QColorDialog>

#define COLOR_POINTCLOUD    (0)
#define COLOR_BACKGROUNG    (1)
#define COLOR_NORMALS       (2)
#define COLOR_BOUNDINGBOX   (3)

const QColor colors[5][10] =
{
    {QColor("#ffffff"), QColor("#e5e5e7"), QColor("#cccccc"), QColor("#9a9a9a"),
     QColor("#7f7f7f"), QColor("#666666"), QColor("#4c4c4c"), QColor("#333333"),
     QColor("#191919"), QColor("#000000")},

    {QColor("#ffd6e7"), QColor("#ffccc7"), QColor("#ffe7ba"), QColor("#ffffb8"),
     QColor("#f4ffb8"), QColor("#d9f7be"), QColor("#b5f5ec"), QColor("#bae7ff"),
     QColor("#d6e4ff"), QColor("#efdbff")},

    {QColor("#ff85c0"), QColor("#ff7875"), QColor("#ffc069"), QColor("#fff566"),
     QColor("#d3f261"), QColor("#95de64"), QColor("#5cdbd3"), QColor("#69c0ff"),
     QColor("#85a5ff"), QColor("#b37feb")},

    {QColor("#f759ab"), QColor("#ff4d4f"), QColor("#ffa940"), QColor("#ffdf3d"),
     QColor("#a0d911"), QColor("#52c41a"), QColor("#13c2c2"), QColor("#1890ff"),
     QColor("#2f54eb"), QColor("#722ed1")},

    {QColor("#9e1068"), QColor("#a8171b"), QColor("#ad4e00"), QColor("#ad8b00"),
     QColor("#5b8c00"), QColor("#006075"), QColor("#006d75"), QColor("#0050b3"),
     QColor("#10239e"), QColor("#391085")}
};

Color::Color(QWidget* parent)
    : CustomDock(parent), ui(new Ui::Color), m_field(""), m_rgb(QColor(255,255,255))
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
                            emit rgb(QColorDialog::getColor(Qt::white, this, tr("select color")));
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
                            emit rgb(colors[row][column]);
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
    connect(this, &Color::rgb, this, &Color::setColorRGB);
    connect(this, &Color::field, this, &Color::setColorField);
}

Color::~Color() { delete ui; }

void Color::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        switch (ui->cbox_type->currentIndex())
        {
        case COLOR_POINTCLOUD:
            if (m_field != "")
            {
                cloud->setCloudColor(m_field);
                printI(QString("Apply cloud[id:%1] point color[axis:%2] done.").arg(cloud->id()).arg(m_field));
            }
            else
            {
                cloud->setCloudColor({m_rgb.red(),m_rgb.green(),m_rgb.blue()});
                printI(QString("Apply cloud[id:%1] point color[r:%2, g:%3, b:%4] done.")
                       .arg(cloud->id()).arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
            }
            m_cloudview->addPointCloud(cloud);
            break;
        case COLOR_BACKGROUNG:
            break;
        case COLOR_NORMALS:
            cloud->setNormalColor({m_rgb.red(),m_rgb.green(),m_rgb.blue()});
            printI(QString("Apply cloud[id:%1] normals color[r:%2, g:%3, b:%4] done.")
                   .arg(cloud->id()).arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
            break;
        case COLOR_BOUNDINGBOX:
            cloud->setBoxColor({m_rgb.red(),m_rgb.green(),m_rgb.blue()});
            printI(QString("Apply cloud[id:%1] box color[r:%2, g:%3, b:%4] done.")
                   .arg(cloud->id()).arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
            break;
        }
    }
}

void Color::reset()
{
    m_rgb = QColor(255,255,255), m_field = "";
    for (auto& cloud : m_cloudtree->getSelectedClouds())
    {
        switch (ui->cbox_type->currentIndex())
        {
        case COLOR_POINTCLOUD:
            m_cloudview->resetPointCloudColor(cloud);
            break;
        case COLOR_NORMALS:
            if (m_cloudview->contains(cloud->normalId()))
                m_cloudview->setPointCloudColor(cloud->normalId(), cloud->normalColor());
            break;
        case COLOR_BOUNDINGBOX:
            if (m_cloudview->contains(cloud->boxId()))
                m_cloudview->setShapeColor(cloud->boxId(), cloud->boxColor());
            break;
        }
        printI(QString("Reset cloud[id:%1] color done.").arg(cloud->id()));
    }
    if (ui->cbox_type->currentIndex() == COLOR_BACKGROUNG)
        m_cloudview->resetBackgroundColor();
}

void Color::setColorRGB(const QColor& rgb)
{
    m_rgb = rgb, m_field = "";
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    switch (ui->cbox_type->currentIndex())
    {
    case COLOR_POINTCLOUD:
        for (auto& cloud : selected_clouds)
            m_cloudview->setPointCloudColor(cloud, {rgb.red(),rgb.green(),rgb.blue()});
        break;
    case COLOR_BACKGROUNG:
        m_cloudview->setBackgroundColor({rgb.red(),rgb.green(),rgb.blue()});
        break;
    case COLOR_NORMALS:
        for (auto& cloud : selected_clouds)
            if (m_cloudview->contains(cloud->normalId()))
                m_cloudview->setPointCloudColor(cloud->normalId(), {rgb.red(),rgb.green(),rgb.blue()});
        break;
    case COLOR_BOUNDINGBOX:
        for (auto& cloud : selected_clouds)
            if (m_cloudview->contains(cloud->boxId()))
                m_cloudview->setShapeColor(cloud->boxId(), {rgb.red(),rgb.green(),rgb.blue()});
        break;
    }
}

void Color::setColorField(const QString& field)
{
    m_field = field, m_rgb = QColor(255,255,255);
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& i : selected_clouds) m_cloudview->setPointCloudColor(i, field);
}
