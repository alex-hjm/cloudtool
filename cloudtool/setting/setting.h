/**
 * @file setting.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-02-26
 */
#ifndef _CT_SETTING_H_
#define _CT_SETTING_H_

#include <QDialog>
#include <QTranslator>
#include <QSettings>
#include "base/console.h"

namespace Ui { class Setting;}

class Setting : public QDialog
{
    Q_OBJECT
public:
    enum Theme { Light, Dark };
    enum Language { English, Chinese };

    explicit Setting(QWidget *parent = nullptr);
    ~Setting();

    void loadSetting();

    void setValue(const QString& key, const QVariant& value);
    QVariant value(const QString& key, const QVariant& defaultValue);
    
private:
    void setTheme(Theme theme);
    void setLanguage(Language language);

signals:
    void logging(ct::LogLevel level, const QString& msg);
    void changeThemeEvent(Theme theme);
    void changeLanguageEvent();

private:
    Ui::Setting *ui;
    QTranslator* m_translator;
    QSettings* m_setting;
    Theme m_theme;
    Language m_language;
};

#endif // _CT_SETTING_H_
