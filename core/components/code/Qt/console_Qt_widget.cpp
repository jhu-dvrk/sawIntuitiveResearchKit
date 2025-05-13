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
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::TeleopEnabledEventHandler,
                                                this, "teleop_enabled");
        interface_required->AddFunction("select_teleop", console.select_teleop);
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::TeleopSelectedEventHandler,
                                                this, "teleop_selected");
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::TeleopUnselectedEventHandler,
                                                this, "teleop_unselected");
        interface_required->AddFunction("set_scale", console.set_scale);
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::ScaleEventHandler,
                                                this, "scale");
        interface_required->AddFunction("emulate_operator_present", console.emulate_operator_present);
        interface_required->AddFunction("emulate_clutch", console.emulate_clutch);
        interface_required->AddFunction("emulate_camera", console.emulate_camera);
    }
    interface_required = AddInterfaceRequired("operator_present");
    if (interface_required) {
        interface_required->AddEventHandlerWrite(&dvrk::console_Qt_widget::OperatorPresentEventHandler,
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


void dvrk::console_Qt_widget::HasTeleOp(const bool & hasTeleOp)
{
    QCBTeleopEnable->setEnabled(hasTeleOp);
    QSBScale->setEnabled(hasTeleOp);
}


void dvrk::console_Qt_widget::SlotTeleopEnable(bool toggle)
{
    console.teleop_enable(toggle);
}


void dvrk::console_Qt_widget::SlotTeleopToggle(void)
{
    console.teleop_enable(!(QCBTeleopEnable->isChecked()));
}


void dvrk::console_Qt_widget::SlotTeleopStart(void)
{
    console.teleop_enable(true);
}


void dvrk::console_Qt_widget::SlotTeleopStop(void)
{
    console.teleop_enable(false);
}


void dvrk::console_Qt_widget::SlotTeleopEnabledEventHandler(bool enabled)
{
    if (enabled) {
        QPBTeleopEnable->setText("Enabled");
        QPBTeleopEnable->setStyleSheet("QPushButton { background-color: rgb(50, 255, 50); border: none }");
    } else {
        QPBTeleopEnable->setText("Disabled");
        QPBTeleopEnable->setStyleSheet("QPushButton { background-color: rgb(255, 100, 100); border: none }");
    }
    QCBTeleopEnable->blockSignals(true);
    QCBTeleopEnable->setChecked(enabled);
    QCBTeleopEnable->blockSignals(false);
}


void dvrk::console_Qt_widget::GetTeleopButtonCheck(const std::string & teleop,
                                                   QPushButton * & button,
                                                   QCheckBox * & check)
{
    auto iter = TeleopButtons.find(teleop);
    // insert new teleop if needed
    if (iter == TeleopButtons.end()) {
        QHBoxLayout * buttonsLayout = new QHBoxLayout;
        button = new QPushButton(teleop.c_str());
        buttonsLayout->addWidget(button);
        buttonsLayout->addStretch();
        check = new QCheckBox("");
        buttonsLayout->addWidget(check);
        QVBTeleops->addLayout(buttonsLayout);
        TeleopButtons[teleop] = std::pair<QPushButton *, QCheckBox *>(button, check);
        connect(check, &QCheckBox::toggled,
                [ = ](bool checked) {
                    if (checked) {
                        SelectTeleopCheck(teleop);
                    } else {
                        UnselectTeleopCheck(teleop);
                    }
                });
    } else {
        button = iter->second.first;
        check = iter->second.second;
    }
}


void dvrk::console_Qt_widget::SlotTeleopSelectedEventHandler(std::string selected)
{
    QPushButton * button;
    QCheckBox * check;
    GetTeleopButtonCheck(selected, button, check);
    button->setStyleSheet("QPushButton { border: none }");
    check->blockSignals(true);
    check->setChecked(true);
    check->blockSignals(false);
}


void dvrk::console_Qt_widget::SlotTeleopUnselectedEventHandler(std::string unselected)
{
    QPushButton * button;
    QCheckBox * check;
    GetTeleopButtonCheck(unselected, button, check);
    button->setStyleSheet("QPushButton { border: none; color: palette(mid) }");
    check->blockSignals(true);
    check->setChecked(false);
    check->blockSignals(false);
}


void dvrk::console_Qt_widget::SlotSetScale(double scale)
{
    console.set_scale(scale);
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
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_T), this, SLOT(SlotTeleopStart()));
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_S), this, SLOT(SlotTeleopStop()));
    // set default to false
    SlotTeleopEnabledEventHandler(false);
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
    // qRegisterMetaType<std::string>("std::string");
    connect(QPBTeleopEnable, SIGNAL(clicked()),
            this, SLOT(SlotTeleopToggle()));
    connect(QCBTeleopEnable, SIGNAL(toggled(bool)),
            this, SLOT(SlotTeleopEnable(bool)));
    connect(this, SIGNAL(SignalTeleopEnabled(bool)),
            this, SLOT(SlotTeleopEnabledEventHandler(bool)));
    connect(this, SIGNAL(SignalTeleopSelected(std::string)),
            this, SLOT(SlotTeleopSelectedEventHandler(std::string)));
    connect(this, SIGNAL(SignalTeleopUnselected(std::string)),
            this, SLOT(SlotTeleopUnselectedEventHandler(std::string)));
    connect(QSBScale, SIGNAL(valueChanged(double)),
            this, SLOT(SlotSetScale(double)));
    connect(this, SIGNAL(SignalScale(double)),
            this, SLOT(SlotScaleEventHandler(double)));
    connect(this, SIGNAL(SignalOperatorPresent(bool)),
            this, SLOT(SlotOperatorPresentEventHandler(bool)));
    connect(this, SIGNAL(signal_clutch(bool)),
            this, SLOT(slot_clutched(bool)));
    connect(this, SIGNAL(SignalCamera(bool)),
            this, SLOT(SlotCameraEventHandler(bool)));
    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)),
            this, SLOT(SlotEnableDirectControl(bool)));
    connect(QRBOperatorPresent, SIGNAL(clicked(bool)),
            this, SLOT(SlotEmulateOperatorPresent(bool)));
    connect(QRBClutch, SIGNAL(clicked(bool)),
            this, SLOT(SlotEmulateClutch(bool)));
    connect(QRBCamera, SIGNAL(clicked(bool)),
            this, SLOT(SlotEmulateCamera(bool)));

    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));
}


void dvrk::console_Qt_widget::TeleopEnabledEventHandler(const bool & _enabled)
{
    emit SignalTeleopEnabled(_enabled);
}


void dvrk::console_Qt_widget::TeleopSelectedEventHandler(const std::string & _selected)
{
    emit SignalTeleopSelected(_selected);
}


void dvrk::console_Qt_widget::TeleopUnselectedEventHandler(const std::string & _unselected)
{
    emit SignalTeleopUnselected(_unselected);
}


void dvrk::console_Qt_widget::SlotScaleEventHandler(double _scale)
{
    QSBScale->setValue(_scale);
}


void dvrk::console_Qt_widget::ScaleEventHandler(const double & scale)
{
    emit SignalScale(scale);
}


void dvrk::console_Qt_widget::SlotOperatorPresentEventHandler(bool _operator_present)
{
    QRBOperatorPresent->setChecked(_operator_present);
}


void dvrk::console_Qt_widget::OperatorPresentEventHandler(const prmEventButton & _button)
{
    if (_button.Type() == prmEventButton::PRESSED) {
        emit SignalOperatorPresent(true);
    } else {
        emit SignalOperatorPresent(false);
    }
}


void dvrk::console_Qt_widget::slot_clutched(bool _clutch)
{
    QRBClutch->setChecked(_clutch);
}


void dvrk::console_Qt_widget::clutch_event_handler(const prmEventButton & _button)
{
    std::cerr << "clutch_event_handler " << _button << std::endl;
    if (_button.Type() == prmEventButton::PRESSED) {
        emit signal_clutch(true);
    } else {
        emit signal_clutch(false);
    }
}


void dvrk::console_Qt_widget::SlotCameraEventHandler(bool _camera)
{
    QRBCamera->setChecked(_camera);
}


void dvrk::console_Qt_widget::SlotEnableDirectControl(bool _toggle)
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


void dvrk::console_Qt_widget::SlotEmulateOperatorPresent(bool _toggle)
{
    prmEventButton event;
    if (_toggle) {
        event.SetType(prmEventButton::PRESSED);
    } else {
        event.SetType(prmEventButton::RELEASED);
    }
    console.emulate_operator_present(event);
}


void dvrk::console_Qt_widget::SlotEmulateClutch(bool toggle)
{
    prmEventButton event;
    if (toggle) {
        event.SetType(prmEventButton::PRESSED);
    } else {
        event.SetType(prmEventButton::RELEASED);
    }
    console.emulate_clutch(event);
}


void dvrk::console_Qt_widget::SlotEmulateCamera(bool toggle)
{
    prmEventButton event;
    if (toggle) {
        event.SetType(prmEventButton::PRESSED);
    } else {
        event.SetType(prmEventButton::RELEASED);
    }
    console.emulate_camera(event);
}


void dvrk::console_Qt_widget::SelectTeleopCheck(const std::string & teleop)
{
    console.select_teleop(teleop);
}


void dvrk::console_Qt_widget::UnselectTeleopCheck(const std::string & teleop)
{
    console.select_teleop(teleop);
}


void dvrk::console_Qt_widget::camera_event_handler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        emit SignalCamera(true);
    } else {
        emit SignalCamera(false);
    }
}
