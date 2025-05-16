/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-17

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// cisst
#include <cisstBuildType.h>
#include <cisstRevision.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsComponentViewer.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/console_Qt_widget.h>

#include <QMessageBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QPushButton>
#include <QScrollBar>
#include <QGroupBox>
#include <QTabWidget>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QShortcut>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QRadioButton>
#include <QCheckBox>
#include <QApplication>

typedef dvrk::console_Qt_widget dvrk_console_Qt_widget;
CMN_IMPLEMENT_SERVICES(dvrk_console_Qt_widget);

dvrk::console_Qt_widget::console_Qt_widget(const std::string & _component_name):
    mtsComponent(_component_name)
{
    mtsInterfaceRequired * interface_required = AddInterfaceRequired("Main");
    if (interface_required) {
        interface_required->AddFunction("teleop_enable", console.teleop_enable);
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::teleop_enabled_event_handler,
                                                this, "teleop_enabled");
        interface_required->AddFunction("select_teleop", console.select_teleop);
        interface_required->AddFunction("unselect_teleop", console.unselect_teleop);
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::teleop_selected_event_handler,
                                                this, "teleop_selected");
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::teleop_unselected_event_handler,
                                                this, "teleop_unselected");
        interface_required->AddFunction("set_scale", console.set_scale);
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::scale_event_handler,
                                                this, "scale");
        interface_required->AddFunction("emulate_operator_present", console.emulate_operator_present);
        interface_required->AddFunction("emulate_clutch", console.emulate_clutch);
        interface_required->AddFunction("emulate_camera", console.emulate_camera);
    }
    interface_required = AddInterfaceRequired("operator_present");
    if (interface_required) {
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::operator_present_event_handler,
                                                this, "Button");
    }
    interface_required = AddInterfaceRequired("clutch");
    if (interface_required) {
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::clutch_event_handler,
                                                 this, "Button");
    }
    interface_required = AddInterfaceRequired("camera");
    if (interface_required) {
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::camera_event_handler,
                                                this, "Button");
    }
    setupUi();
}


void dvrk::console_Qt_widget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}


void dvrk::console_Qt_widget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    if (!parent()) {
        show();
    }
}


void dvrk::console_Qt_widget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}


void dvrk::console_Qt_widget::slot_teleop_enable(bool _toggle)
{
    console.teleop_enable(_toggle);
}


void dvrk::console_Qt_widget::slot_teleop_toggle(void)
{
    console.teleop_enable(!(QCBTeleopEnable->isChecked()));
}


void dvrk::console_Qt_widget::slot_teleop_start(void)
{
    console.teleop_enable(true);
}


void dvrk::console_Qt_widget::slot_teleop_stop(void)
{
    console.teleop_enable(false);
}


void dvrk::console_Qt_widget::slot_teleop_enabled_event_handler(bool _enabled)
{
    if (_enabled) {
        QPBTeleopEnable->setText("Enabled");
        QPBTeleopEnable->setStyleSheet("QPushButton { background-color: rgb(50, 255, 50); border: none }");
    } else {
        QPBTeleopEnable->setText("Disabled");
        QPBTeleopEnable->setStyleSheet("QPushButton { background-color: rgb(255, 100, 100); border: none }");
    }
    QCBTeleopEnable->blockSignals(true);
    QCBTeleopEnable->setChecked(_enabled);
    QCBTeleopEnable->blockSignals(false);
}


void dvrk::console_Qt_widget::get_teleop_button_check(const QString & teleop,
                                                      QPushButton * & button,
                                                      QCheckBox * & check)
{
    auto iter = m_teleop_buttons.find(teleop);
    // insert new teleop if needed
    if (iter == m_teleop_buttons.end()) {
        QHBoxLayout * buttonsLayout = new QHBoxLayout;
        button = new QPushButton(teleop);
        buttonsLayout->addWidget(button);
        buttonsLayout->addStretch();
        check = new QCheckBox("");
        buttonsLayout->addWidget(check);
        QVBTeleops->addLayout(buttonsLayout);
        m_teleop_buttons[teleop] = std::pair<QPushButton *, QCheckBox *>(button, check);
        connect(check, &QCheckBox::toggled,
                [ = ](bool checked) {
                    if (checked) {
                        select_teleop_check(teleop);
                    } else {
                        unselect_teleop_check(teleop);
                    }
                });
    } else {
        button = iter->second.first;
        check = iter->second.second;
    }
}


void dvrk::console_Qt_widget::slot_teleop_selected_event_handler(const QString & _selected)
{
    QPushButton * button;
    QCheckBox * check;
    get_teleop_button_check(_selected, button, check);
    button->setStyleSheet("QPushButton { border: none }");
    check->blockSignals(true);
    check->setChecked(true);
    check->blockSignals(false);
}


void dvrk::console_Qt_widget::slot_teleop_unselected_event_handler(const QString & _unselected)
{
    QPushButton * button;
    QCheckBox * check;
    get_teleop_button_check(_unselected, button, check);
    button->setStyleSheet("QPushButton { border: none; color: palette(mid) }");
    check->blockSignals(true);
    check->setChecked(false);
    check->blockSignals(false);
}


void dvrk::console_Qt_widget::slot_set_scale(double _scale)
{
    console.set_scale(_scale);
}


void dvrk::console_Qt_widget::setupUi(void)
{
    QVBoxLayout * mainLayout = new QVBoxLayout();
    mainLayout->setContentsMargins(0, 0, 0, 0);
    this->setLayout(mainLayout);

    QGroupBox * teleopBox = new QGroupBox("Tele operation");
    mainLayout->addWidget(teleopBox);
    QVBoxLayout * teleopLayout = new QVBoxLayout();
    teleopLayout->setContentsMargins(2, 2, 2, 2);
    teleopBox->setLayout(teleopLayout);

    QHBoxLayout * teleopEnableLayout = new QHBoxLayout;
    QPBTeleopEnable = new QPushButton("");
    teleopEnableLayout->addWidget(QPBTeleopEnable);
    teleopEnableLayout->addStretch();
    QCBTeleopEnable = new QCheckBox("");
    QPBTeleopEnable->setToolTip("ctrl + T to start\nctrl + S to stop");
    QCBTeleopEnable->setToolTip("ctrl + T to start\nctrl + S to stop");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_T), this, SLOT(slot_teleop_start()));
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_S), this, SLOT(slot_teleop_stop()));
    // set default to false
    slot_teleop_enabled_event_handler(false);
    teleopEnableLayout->addWidget(QCBTeleopEnable);
    teleopLayout->addLayout(teleopEnableLayout);

    QSBScale = new QDoubleSpinBox();
    QSBScale->setRange(0.1, 1.0);
    QSBScale->setSingleStep(0.1);
    QSBScale->setPrefix("scale ");
    QSBScale->setValue(mtsIntuitiveResearchKit::TeleOperationPSM::Scale);
    teleopLayout->addWidget(QSBScale);
    QVBTeleops = new QVBoxLayout();
    teleopLayout->addLayout(QVBTeleops);

    QGroupBox * inputsBox = new QGroupBox("Inputs");
    mainLayout->addWidget(inputsBox);
    QVBoxLayout * inputsLayout = new QVBoxLayout();
    inputsLayout->setContentsMargins(2, 2, 2, 2);
    inputsBox->setLayout(inputsLayout);
    QRBOperatorPresent = new QRadioButton("Operator");
    QRBOperatorPresent->setAutoExclusive(false);
    QRBOperatorPresent->setChecked(false);
    QRBOperatorPresent->setEnabled(false);
    inputsLayout->addWidget(QRBOperatorPresent);
    QRBClutch = new QRadioButton("Clutch");
    QRBClutch->setAutoExclusive(false);
    QRBClutch->setChecked(false);
    QRBClutch->setEnabled(false);
    inputsLayout->addWidget(QRBClutch);
    QRBCamera = new QRadioButton("Camera");
    QRBCamera->setAutoExclusive(false);
    QRBCamera->setChecked(false);
    QRBCamera->setEnabled(false);
    inputsLayout->addWidget(QRBCamera);

    QCBEnableDirectControl = new QCheckBox("Direct control");
    QCBEnableDirectControl->setToolTip("Allows to emulate console events with buttons");
    mainLayout->addWidget(QCBEnableDirectControl);

    // buttons
    connect(QPBTeleopEnable, SIGNAL(clicked()),
            this, SLOT(slot_teleop_toggle()));
    connect(QCBTeleopEnable, SIGNAL(toggled(bool)),
            this, SLOT(slot_teleop_enable(bool)));
    connect(this, SIGNAL(signal_teleop_enabled(bool)),
            this, SLOT(slot_teleop_enabled_event_handler(bool)));
    connect(this, SIGNAL(signal_teleop_selected(const QString &)),
            this, SLOT(slot_teleop_selected_event_handler(const QString &)));
    connect(this, SIGNAL(signal_teleop_unselected(const QString &)),
            this, SLOT(slot_teleop_unselected_event_handler(const QString &)));
    connect(QSBScale, SIGNAL(valueChanged(double)),
            this, SLOT(slot_set_scale(double)));
    connect(this, SIGNAL(signal_scale(double)),
            this, SLOT(slot_scale_event_handler(double)));
    connect(this, SIGNAL(signal_operator_present(bool)),
            this, SLOT(slot_operator_present_event_handler(bool)));
    connect(this, SIGNAL(signal_clutch(bool)),
            this, SLOT(slot_clutched(bool)));
    connect(this, SIGNAL(signal_camera(bool)),
            this, SLOT(slot_camera_event_handler(bool)));
    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)),
            this, SLOT(slot_enable_direct_control(bool)));
    connect(QRBOperatorPresent, SIGNAL(clicked(bool)),
            this, SLOT(slot_emulate_operator_present(bool)));
    connect(QRBClutch, SIGNAL(clicked(bool)),
            this, SLOT(slot_emulate_clutch(bool)));
    connect(QRBCamera, SIGNAL(clicked(bool)),
            this, SLOT(slot_emulate_camera(bool)));

    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));
}


void dvrk::console_Qt_widget::teleop_enabled_event_handler(const bool & _enabled)
{
    emit signal_teleop_enabled(_enabled);
}


void dvrk::console_Qt_widget::teleop_selected_event_handler(const std::string & _selected)
{
    emit signal_teleop_selected(_selected.c_str());
}


void dvrk::console_Qt_widget::teleop_unselected_event_handler(const std::string & _unselected)
{
    emit signal_teleop_unselected(_unselected.c_str());
}


void dvrk::console_Qt_widget::slot_scale_event_handler(double _scale)
{
    QSBScale->setValue(_scale);
}


void dvrk::console_Qt_widget::scale_event_handler(const double & _scale)
{
    emit signal_scale(_scale);
}


void dvrk::console_Qt_widget::slot_operator_present_event_handler(bool _operator_present)
{
    QRBOperatorPresent->setChecked(_operator_present);
}


void dvrk::console_Qt_widget::operator_present_event_handler(const prmEventButton & _button)
{
    if (_button.Type() == prmEventButton::PRESSED) {
        emit signal_operator_present(true);
    } else {
        emit signal_operator_present(false);
    }
}


void dvrk::console_Qt_widget::slot_clutched(bool _clutch)
{
    QRBClutch->setChecked(_clutch);
}


void dvrk::console_Qt_widget::clutch_event_handler(const prmEventButton & _button)
{
    if (_button.Type() == prmEventButton::PRESSED) {
        emit signal_clutch(true);
    } else {
        emit signal_clutch(false);
    }
}


void dvrk::console_Qt_widget::slot_camera_event_handler(bool _camera)
{
    QRBCamera->setChecked(_camera);
}


void dvrk::console_Qt_widget::slot_enable_direct_control(bool _toggle)
{
    if (_toggle) {
        int answer = QMessageBox::warning(this, tr("dvrk::console_Qt_widget"),
                                          tr("Mixing real and emulated console events can lead to inconsistent states.\nAre you sure you want to continue?"),
                                          QMessageBox::No | QMessageBox::Yes, // options
                                          QMessageBox::No // default
                                          );
        if (answer == QMessageBox::No) {
            return;
        }
    }
    QRBOperatorPresent->setEnabled(_toggle);
    QRBClutch->setEnabled(_toggle);
    QRBCamera->setEnabled(_toggle);
}


void dvrk::console_Qt_widget::slot_emulate_operator_present(bool _toggle)
{
    prmEventButton event;
    if (_toggle) {
        event.SetType(prmEventButton::PRESSED);
    } else {
        event.SetType(prmEventButton::RELEASED);
    }
    console.emulate_operator_present(event);
}


void dvrk::console_Qt_widget::slot_emulate_clutch(bool _toggle)
{
    prmEventButton event;
    if (_toggle) {
        event.SetType(prmEventButton::PRESSED);
    } else {
        event.SetType(prmEventButton::RELEASED);
    }
    console.emulate_clutch(event);
}


void dvrk::console_Qt_widget::slot_emulate_camera(bool _toggle)
{
    prmEventButton event;
    if (_toggle) {
        event.SetType(prmEventButton::PRESSED);
    } else {
        event.SetType(prmEventButton::RELEASED);
    }
    console.emulate_camera(event);
}


void dvrk::console_Qt_widget::select_teleop_check(const QString & _teleop)
{
    console.select_teleop(_teleop.toStdString());
}


void dvrk::console_Qt_widget::unselect_teleop_check(const QString & _teleop)
{
    console.unselect_teleop(_teleop.toStdString());
}


void dvrk::console_Qt_widget::camera_event_handler(const prmEventButton & _button)
{
    if (_button.Type() == prmEventButton::PRESSED) {
        emit signal_camera(true);
    } else {
        emit signal_camera(false);
    }
}
