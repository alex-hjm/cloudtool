/**
 * @file registration.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#ifndef CT_TOOL_REGISTRATION_H
#define CT_TOOL_REGISTRATION_H

#include "base/customdock.h"
#include "modules/registration.h"
#include "cloudtool/tool/descriptor.h"

#include <QThread>

namespace Ui
{
    class Registration;
}

class Registration : public ct::CustomDock
{
    Q_OBJECT

public:
    explicit Registration(QWidget* parent = nullptr);
    ~Registration();

    void setDescriptor(Descriptor* desciptor)
    {
        m_descriptor = desciptor;
        if (desciptor) connect(desciptor, &Descriptor::destroyed, [=] {m_descriptor = nullptr;});
    }
    void setTarget();
    void setSource();
    void preview();
    void add();
    void apply();
    void reset();

public slots:
    void correspondenceEstimationResult(const ct::CorrespondencesPtr& corr, float time);

private:
    Ui::Registration* ui;
    QThread m_thread;
    Descriptor* m_descriptor;
    ct::Registration* m_reg;
    ct::Cloud::Ptr m_target_cloud;
    ct::Cloud::Ptr m_source_cloud;
};

#endif // REGISTRATIONS_H
