/**
 * @file color.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-06
 */
#include "color.h"
#include "ui_color.h"

#include "setting/setting.h"
#include <QColorDialog>

static const QColor colors[6][10] =
{
    {"#FFCCCC", "#FFE5CC", "#FFFFCC", "#E5FFCC", "#CCFFCC", "#CCFFE5", "#CCFFFF", "#CCE5FF", "#CCCCFF", "#E5CCFF"},
    {"#FF9999", "#FFCC99", "#FFFF99", "#CCFF99", "#99FF99", "#99FFCC", "#99FFFF", "#99CCFF", "#9999FF", "#CC99FF"},
    {"#FF6666", "#FFB266", "#FFFF66", "#B2FF66", "#66FF66", "#66FFB2", "#66FFFF", "#66B2FF", "#6666FF", "#B266FF"},
    {"#FF3333", "#FF9933", "#FFFF33", "#99FF33", "#33FF33", "#33FF99", "#33FFFF", "#3399FF", "#3333FF", "#9933FF"},
    {"#FF0000", "#FF8000", "#FFFF00", "#80FF00", "#00FF00", "#00FF80", "#00FFFF", "#0080FF", "#0000FF", "#7F00FF"},
    {"#FFFFFF", "#E6E6E6", "#CCCCCC", "#B3B3B3", "#999999", "#808080", "#666666", "#4D4D4D", "#000000", "#000000"},
};

enum ColorIndex {
    PointCloud = 0,
    BoundingBox,
    Background
};

Color::Color(QWidget* parent): CustomDock(parent), 
        ui(new Ui::Color)
{
    ui->setupUi(this);
    ui->cbox_type->setCurrentIndex(0);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Color::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Color::reset);

    ui->gridLayout->setSpacing(0);
    for (int row = 0; row < 6; row++) {
        for (int column = 0; column < 10; column++) {
            QPushButton* btn = new QPushButton();
            btn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            btn->setFixedHeight(20);
            if (row == 5 && column == 9) {
                btn->setText("+");
                btn->setStyleSheet(tr("QPushButton{border:0px;background-color:transparent;}"
                                      "QPushButton:pressed{background-color:lightgray;}"));
                connect(btn, &QPushButton::clicked, [=] { setColor(QColorDialog::getColor(Qt::white, this, 
                                                          tr("Select color"))); });
            } else {
                auto& color = colors[row][column];
                btn->setToolTip(tr("rgb(%1, %2, %3)").arg(color.red()).arg(color.green()).arg(color.blue()));
                btn->setStyleSheet(tr("QPushButton{border:0px;background-color:rgb(%1, %2, %3);}"
                                    "QPushButton:pressed{background-color:lightgray;}")
                                    .arg(color.red()).arg(color.green()).arg(color.blue()));
                connect(btn, &QPushButton::clicked, [=] { setColor(color); });
            }
            ui->gridLayout->addWidget(btn, row, column);
        }
    }
}

Color::~Color() 
{ 
    delete ui; 
}



void Color::apply()
{
    auto selectedClouds = m_cloudlist->getSelectedClouds();
    switch (ui->cbox_type->currentIndex()) {
    case PointCloud:
        for (auto& cloud : selectedClouds) {
            cloud->setColor({m_rgb.red(),m_rgb.green(),m_rgb.blue()});
            m_cloudview->addCloud(cloud);
            m_console->logging(ct::LOG_INFO, tr("Apply cloud color done. cloud id: %1, rgb(%2, %3, %4)")
                        .arg(cloud->id()).arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
        }
        break;
    case BoundingBox:
        for (auto& cloud : selectedClouds) {
            cloud->setBBoxColor({m_rgb.red(),m_rgb.green(),m_rgb.blue()});
            m_cloudview->addCloudBBox(cloud);
            m_console->logging(ct::LOG_INFO, tr("Apply cloud bbox color done. cloud id: %1, rgb(%2, %3, %4)")
                        .arg(cloud->id()).arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
        }
        break;
    }
}

void Color::reset()
{
    auto selectedClouds = m_cloudlist->getSelectedClouds();
    switch (ui->cbox_type->currentIndex()) {
    case PointCloud:
        for (auto& cloud : selectedClouds) {
            m_cloudview->resetCloudColor(cloud);
            m_console->logging(ct::LOG_INFO, tr("Reset cloud color done. cloud id: %1").arg(cloud->id()));
        }
        break;
    case BoundingBox:
        for (auto& cloud : selectedClouds) {
            m_cloudview->setShapeColor(cloud->bboxId(), cloud->bboxColor());
            m_console->logging(ct::LOG_INFO, tr("Reset cloud bbox color done. cloud id: %1").arg(cloud->id()));
        }
        break;
    case Background:
        switch (Setting::theme()) {
        case Setting::Light:
            m_cloudview->setBackgroundColor(ct::Color::Light);
            break;
        case Setting::Dark:
            m_cloudview->setBackgroundColor(ct::Color::Dark);
            break;
        }
        break;
    }
}

void Color::handleLanguageChanged()
{
    ui->retranslateUi(this);
}


void Color::setColor(const QColor& rgb)
{
    m_rgb = rgb;
    auto selectedClouds = m_cloudlist->getSelectedClouds();
    switch (ui->cbox_type->currentIndex()) {
    case PointCloud:
        for (auto& cloud : selectedClouds)
            m_cloudview->setCloudColor(cloud, {rgb.red(),rgb.green(),rgb.blue()});
        break;
    case BoundingBox:
        for (auto& cloud : selectedClouds)
            m_cloudview->setShapeColor(cloud->bboxId(), {rgb.red(),rgb.green(),rgb.blue()});
        break;
    case Background:
        m_cloudview->setBackgroundColor({rgb.red(),rgb.green(),rgb.blue()});
        break;
    }
}

