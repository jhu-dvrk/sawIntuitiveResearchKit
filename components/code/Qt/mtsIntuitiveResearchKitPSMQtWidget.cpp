/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-08-07

  (C) Copyright 2019-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// Qt include
#include <QMessageBox>
#include <QComboBox>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitToolTypes.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitPSMQtWidget, mtsComponent, std::string);

mtsIntuitiveResearchKitPSMQtWidget::mtsIntuitiveResearchKitPSMQtWidget(const std::string & componentName, double periodInSeconds):
    mtsIntuitiveResearchKitArmQtWidget(componentName, periodInSeconds)
{
    CMN_ASSERT(InterfaceRequired);
    InterfaceRequired->AddFunction("jaw/measured_js", Jaw.measured_js);
    InterfaceRequired->AddFunction("jaw/configuration_js", Jaw.configuration_js);
    InterfaceRequired->AddFunction("jaw/move_jp", Jaw.move_jp);
    InterfaceRequired->AddFunction("tool_list_size", tool_list_size);
    InterfaceRequired->AddFunction("tool_name", tool_name);
    InterfaceRequired->AddFunction("tool_full_description", tool_full_description);
    InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSMQtWidget::ToolTypeEventHandler,
                                            this, "tool_type");
    InterfaceRequired->AddEventHandlerVoid(&mtsIntuitiveResearchKitPSMQtWidget::ToolTypeRequestEventHandler,
                                           this, "tool_type_request");
    InterfaceRequired->AddFunction("set_tool_type", set_tool_type);
}

void mtsIntuitiveResearchKitPSMQtWidget::setupUiDerived(void)
{
    // jaw effort plot
    const QColor baseColor = palette().color(QPalette::Base);
    const QColor textColor = palette().color(QPalette::Text);

    QVP2DJaw = new vctPlot2DOpenGLQtWidget();
    QVP2DJaw->SetBackgroundColor(vct3(baseColor.redF(), baseColor.greenF(), baseColor.blueF()));
    QVP2DJaw->resize(QVP2DJaw->sizeHint());
    QVP2DJaw->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    QVP2DJaw->SetContinuousExpandYResetSlot();
    EffortLayout->addWidget(QVP2DJaw);

    m_scale_jaw = QVP2DJaw->AddScale("jaw/measured_jf");
    m_signal_jaw = m_scale_jaw->AddSignal("jaw/measured_jf");
    m_signal_jaw->SetColor(vct3(textColor.redF(), textColor.greenF(), textColor.blueF()));
    m_signal_jaw_zero = m_scale_jaw->AddSignal("zero");
    m_signal_jaw_zero->SetColor(vct3(0.5));
    // to display range in plot area
    QVP2DJaw->SetDisplayYRangeScale(m_scale_jaw);
    QVP2DJaw->setToolTip("Jaw torque");

    QFont font;
    font.setBold(true);

    QFrame * jawFrame = new QFrame();
    jawFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    MainLayout->addWidget(jawFrame);

    QHBoxLayout * jawLayout = new QHBoxLayout;
    jawLayout->setContentsMargins(1, 1, 1, 1);
    jawFrame->setLayout(jawLayout);

    QLabel * jawTitle = new QLabel("Jaw");
    jawTitle->setFont(font);
    jawLayout->addWidget(jawTitle);
    QLEJawPosition = new QLineEdit();
    jawLayout->addWidget(QLEJawPosition);
    QLEJawVelocity = new QLineEdit();
    jawLayout->addWidget(QLEJawVelocity);
    QLEJawEffort = new QLineEdit();
    jawLayout->addWidget(QLEJawEffort);

    jawLayout->addStretch();

    QPJSJaw = new prmPositionJointSetQtWidget();
    QPJSJaw->setupUi();
    QPJSJaw->measured_js = &(Jaw.measured_js);
    QPJSJaw->configuration_js = &(Jaw.configuration_js);
    QPJSJaw->move_jp = &(Jaw.move_jp);
    QPJSJaw->SetPrismaticRevoluteFactors(1.0 / cmn_mm, cmn180_PI);
    jawLayout->addWidget(QPJSJaw);
    QPJSJaw->hide();

    QFrame * toolFrame = new QFrame();
    toolFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    MainLayout->addWidget(toolFrame);

    QHBoxLayout * toolLayout = new QHBoxLayout;
    toolLayout->setContentsMargins(1, 1, 1, 1);
    toolFrame->setLayout(toolLayout);

    // status
    QLabel * titleTool = new QLabel("Tool");
    titleTool->setFont(font);
    toolLayout->addWidget(titleTool);
    QLEToolType = new QLineEdit("");
    QLEToolType->setReadOnly(true);
    toolLayout->addWidget(QLEToolType);

    // set tool type
    QCBToolOptions = new QComboBox();
    QCBToolOptions->setEnabled(false);
    QCBToolOptions->addItem("Select tool type");
    toolLayout->addWidget(QCBToolOptions);

    toolLayout->addStretch();

    // setup Qt Connection
    connect(this, SIGNAL(SignalToolType(QString)),
            this, SLOT(SlotToolTypeEventHandler(QString)));
    connect(this, SIGNAL(SignalToolTypeRequest(void)),
            this, SLOT(SlotToolTypeRequestEventHandler(void)));
    connect(QCBToolOptions, SIGNAL(activated(QString)),
            this, SLOT(SlotToolTypeSelected(QString)));
}

void mtsIntuitiveResearchKitPSMQtWidget::timerEventDerived(void)
{
    // refresh list of tools if empty and connected
    if ((QCBToolOptions->count() == 1) && tool_list_size.IsValid()) {
        size_t nb_tools;
        tool_list_size(nb_tools);
        for (size_t index = 0;
             index < nb_tools;
             ++index) {
            std::string description;
            tool_full_description(index, description);
            QCBToolOptions->addItem(description.c_str());
        }
    }

    // get jaw data
    Jaw.measured_js(m_jaw_measured_js);

    QString text;
    if (m_jaw_measured_js.Position().size() > 0) {
        text.setNum(m_jaw_measured_js.Position().at(0) * cmn180_PI, 'f', 3);
        QLEJawPosition->setText(text);
    } else {
        QLEJawPosition->setText("");
    }
    if (m_jaw_measured_js.Velocity().size() > 0) {
        text.setNum(m_jaw_measured_js.Velocity().at(0) * cmn180_PI, 'f', 3);
        QLEJawVelocity->setText(text);
    } else {
        QLEJawVelocity->setText("");
    }
    if (m_jaw_measured_js.Effort().size() > 0) {
        m_signal_jaw->AppendPoint(vctDouble2(m_jaw_measured_js.Timestamp(),
                                             -(cmn180_PI) * m_jaw_measured_js.Effort().at(0)));
        m_signal_jaw_zero->AppendPoint(vctDouble2(m_jaw_measured_js.Timestamp(), 0.0));
        QVP2DJaw->update();
        text.setNum(m_jaw_measured_js.Effort().at(0), 'f', 3);
        QLEJawEffort->setText(text);
    } else {
        QLEJawEffort->setText("");
    }
}

void mtsIntuitiveResearchKitPSMQtWidget::SetDirectControl(const bool direct)
{
    mtsIntuitiveResearchKitArmQtWidget::SetDirectControl(direct);
    if (direct) {
        QPJSJaw->show();
        QPJSJaw->setEnabled(direct);
        QPJSJaw->Read();

    } else {
        QPJSJaw->hide();
    }
}

void mtsIntuitiveResearchKitPSMQtWidget::SlotToolTypeEventHandler(QString toolType)
{
    QPalette palette;
    palette.setColor(QPalette::Base, this->palette().color(QPalette::Base));
    QLEToolType->setPalette(palette);
    QLEToolType->setText(toolType);
}

void mtsIntuitiveResearchKitPSMQtWidget::SlotToolTypeRequestEventHandler(void)
{
    QPalette palette;
    palette.setColor(QPalette::Base, QColor(255, 100, 100));
    QLEToolType->setPalette(palette);
    QLEToolType->setText("Please select a tool type");
    QCBToolOptions->setEnabled(true);
}

void mtsIntuitiveResearchKitPSMQtWidget::SlotToolTypeSelected(QString toolDescription)
{
    // first line is just a label
    if (QCBToolOptions->currentIndex() == 0) {
        return;
    }
    // otherwise, find as much info as possible
    std::string toolName;
    tool_name(static_cast<size_t>(QCBToolOptions->currentIndex() - 1), toolName);
    std::string message = "Please confirm that the tool inserted matches:\n"
        + toolName + "\n"
        + toolDescription.toStdString();
    int answer = QMessageBox::warning(this, "mtsIntuitiveResearchKitPSMQtWidget",
                                      message.c_str(),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        set_tool_type(toolName);
    }
}

void mtsIntuitiveResearchKitPSMQtWidget::ToolTypeEventHandler(const std::string & toolType)
{
    emit SignalToolType(QString(toolType.c_str()));
}

void mtsIntuitiveResearchKitPSMQtWidget::ToolTypeRequestEventHandler(void)
{
    emit SignalToolTypeRequest();
}
