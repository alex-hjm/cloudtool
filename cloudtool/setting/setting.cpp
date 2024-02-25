/**
 * @file setting.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-02-26
 */
#include "setting.h"
#include "ui_setting.h"

#include <QFile>

#define THEME_LIGHT_FILE    ":/res/theme/light.qss"
#define THEME_DARK_FILE     ":/res/theme/dark.qss"
#define LANGUAGE_CN_FILE    ":/res/trans/zh_CN.qm"

Setting::Setting(QWidget *parent) : QDialog(parent), 
    ui(new Ui::Setting),
    m_translator(new QTranslator(this)),
    m_setting(new QSettings(PROJECT_NAME, PROJECT_NAME)),
    m_theme(Light),
    m_language(English)
{
    ui->setupUi(this);
    ui->listWidget->setCurrentRow(0);

    connect(ui->rbtn_light, &QRadioButton::clicked, [=](bool check) { if(check) setTheme(Light); });
    connect(ui->rbtn_dark, &QRadioButton::clicked, [=](bool check) { if(check) setTheme(Dark); });
    connect(ui->rbtn_en, &QRadioButton::clicked, [=](bool check) { if(check) setLanguage(English); });
    connect(ui->rbtn_cn, &QRadioButton::clicked, [=](bool check) { if(check) setLanguage(Chinese); });

    connect(ui->btn_saveTheme, &QPushButton::clicked, this, [=] {
        m_setting->setValue("theme", static_cast<int>(m_theme));
        logging(ct::LOG_INFO, "Success to save theme setting.");
    });

    connect(ui->btn_saveLanguage, &QPushButton::clicked, this, [=] {
        m_setting->setValue("language", static_cast<int>(m_language));
        logging(ct::LOG_INFO, "Success to save language setting.");
    });
}

Setting::~Setting() 
{ 
    delete ui; 
}

void Setting::loadSetting()
{
    Theme theme = static_cast<Theme>(m_setting->value("theme", m_theme).toInt());
    Language language = static_cast<Language>(m_setting->value("language", m_language).toInt());
    this->setTheme(theme);
    this->setLanguage(language);
}

void Setting::setTheme(Theme theme)
{
    QFile qss;
    switch (theme) {
    case Light:
        ui->rbtn_light->setChecked(true);
        qss.setFileName(THEME_LIGHT_FILE);
        break;
    case Dark:
        ui->rbtn_dark->setChecked(true);
        qss.setFileName(THEME_DARK_FILE);
        break;
    }
    m_theme = theme;
    qss.open(QFile::ReadOnly);
    qApp->setStyleSheet(qss.readAll());
    qss.close();
    emit changeThemeEvent(theme);
}

void Setting::setLanguage(Language language)
{
    switch (language) {
    case English:
        ui->rbtn_en->setChecked(true);
        qApp->removeTranslator(m_translator);
        break;
    case Chinese: 
        ui->rbtn_cn->setChecked(true);
        m_translator->load(LANGUAGE_CN_FILE);
        qApp->installTranslator(m_translator);
        break;
    }
    m_language = language;
    ui->retranslateUi(this);
    emit changeLanguageEvent();
}
