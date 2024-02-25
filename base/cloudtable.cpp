/**
 * @file cloudtable.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-29
 */

#include "cloudtable.h"

#include <QHeaderView>
#include <QLabel>
#include <QSpinBox>

CT_BEGIN_NAMESPACE

CloudTable::CloudTable(QWidget* parent) : QTableWidget(parent),
    m_properties({"Id", "PointNum", "Resolution", "PointSize", "Opacity"}),
    m_labels({"Property", "Value"})
{
    this->setColumnCount(m_labels.size());
    this->setRowCount(m_properties.size());

    this->setHorizontalHeaderLabels(m_labels);
    this->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeMode::Stretch);
    this->horizontalHeader()->setStretchLastSection(true);

    this->verticalHeader()->setHidden(true);

    this->setShowGrid(false);
    this->setEditTriggers(QAbstractItemView::NoEditTriggers);
    this->setSelectionMode(QAbstractItemView::NoSelection);
    
    for (int i = 0; i < m_properties.size(); i++) {
        QTableWidgetItem* item(new QTableWidgetItem(m_properties[i]));
        item->setTextAlignment(Qt::AlignCenter);
        this->setItem(i, 0, item);
    }
}

void CloudTable::handleSelectCloud(const Cloud::Ptr& cloud)
{
    for (int i = 0; i < m_properties.size(); i++) {
        this->removeCellWidget(i, 1);
        if (cloud == nullptr) continue;
        if (m_properties[i] == "Id") {
            QLabel* id = new QLabel(cloud->id(), this);
            id->setAlignment(Qt::AlignCenter);
            this->setCellWidget(i, 1, id);
        } else if (m_properties[i] == "PointNum") {
            QLabel* pointNum = new QLabel(QString::number(cloud->pointNum()), this);
            pointNum->setAlignment(Qt::AlignCenter);
            this->setCellWidget(i, 1, pointNum);
        } else if (m_properties[i] == "Resolution") {
            QLabel* resolution = new QLabel(QString::number(cloud->resolution(), 'f', 8), this);
            resolution->setAlignment(Qt::AlignCenter);
            this->setCellWidget(i, 1, resolution);
        } else if (m_properties[i] == "PointSize") {
            QSpinBox* pointSize = new QSpinBox(this);
            pointSize->setValue(cloud->pointSize());
            pointSize->setRange(1, 99);
            pointSize->setFrame(false);
            pointSize->setButtonSymbols(QAbstractSpinBox::NoButtons);
            pointSize->setAlignment(Qt::AlignCenter);
            connect(pointSize, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=](int value) {
                        cloud->setPointSize(value);
                        updateCloudEvent(cloud);
                    });
            this->setCellWidget(i, 1, pointSize);
        } else if (m_properties[i] == "Opacity") {
            QDoubleSpinBox* opacity = new QDoubleSpinBox(this);
            opacity->setValue(cloud->opacity());
            opacity->setRange(0, 1);
            opacity->setSingleStep(0.1);
            opacity->setDecimals(1);
            opacity->setFrame(false);
            opacity->setButtonSymbols(QAbstractSpinBox::NoButtons);
            opacity->setAlignment(Qt::AlignCenter);
            connect(opacity, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value) {
                        cloud->setOpacity(value);
                        updateCloudEvent(cloud);
                    });
            this->setCellWidget(i, 1, opacity);
        }
    }
}

CT_END_NAMESPACE
