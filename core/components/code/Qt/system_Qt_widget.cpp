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
#include <sawIntuitiveResearchKit/system_Qt_widget.h>

#include <QMessageBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QPushButton>
#include <QGroupBox>
#include <QTabWidget>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QShortcut>
#include <QSlider>

typedef dvrk::system_Qt_widget dvrk_system_Qt_widget;
CMN_IMPLEMENT_SERVICES(dvrk_system_Qt_widget);

dvrk::system_Qt_widget::system_Qt_widget(const std::string & _component_name):
    mtsComponent(_component_name)
{
    QMMessage = new mtsMessageQtWidget();

    mtsInterfaceRequired * interface_required = AddInterfaceRequired("Main");
    if (interface_required) {
        QMMessage->SetInterfaceRequired(interface_required);
        interface_required->AddFunction("power_off", system.power_off);
        interface_required->AddFunction("power_on", system.power_on);
        interface_required->AddFunction("home", system.home);
        interface_required->AddEventHandlerWrite(&dvrk::system_Qt_widget::arm_current_state_event_handler,
                                                 this, "arm_current_state");
        interface_required->AddFunction("set_volume", system.set_volume);
        interface_required->AddEventHandlerWrite(&dvrk::system_Qt_widget::volume_event_handler,
                                                 this, "volume");
    }
    setupUi();
}


void dvrk::system_Qt_widget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}


void dvrk::system_Qt_widget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    if (!parent()) {
        show();
    }

    // warning if not compiled in Release mode
    if (std::string(CISST_BUILD_TYPE) != "Release") {
        std::string message;
        message.append("Warning:\n");
        message.append(" It seems that \"cisst\" has not been compiled in\n");
        message.append(" Release mode.  Make sure your CMake configuration\n");
        message.append(" or catkin profile is configured to compile in\n");
        message.append(" Release mode for better performance and stability");

        QMessageBox * msgBox = new QMessageBox(this);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->setStandardButtons(QMessageBox::Ok);
        msgBox->setWindowTitle("Warning");
        msgBox->setText(message.c_str());
        msgBox->setModal(true);
        msgBox->show();
    }

    // warning if running in calibration mode
    bool calibration_mode;
    system.calibration_mode(calibration_mode);
    if (calibration_mode) {
        std::string message;
        message.append("Warning:\n");
        message.append(" You're running the dVRK system in calibration mode.\n");
        message.append(" You should only do this if you are currently calibrating\n");
        message.append(" potentiometers.");

        QMessageBox * msgBox = new QMessageBox(this);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->setStandardButtons(QMessageBox::Ok);
        msgBox->setWindowTitle("Warning");
        msgBox->setText(message.c_str());
        msgBox->setModal(true);
        msgBox->show();
    }
}


void dvrk::system_Qt_widget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}


void dvrk::system_Qt_widget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("dvrk::system_Qt_widget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes, // options
                                      QMessageBox::Yes // default
                                      );
    if (answer == QMessageBox::Yes) {
        event->accept();
        this->hide();
        // send clean power off message and wait a bit
        system.power_off();
        osaSleep(1.0 * cmn_s);
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}


void dvrk::system_Qt_widget::SlotPowerOff(void)
{
    system.power_off();
}


void dvrk::system_Qt_widget::SlotPowerOn(void)
{
    system.power_on();
}


void dvrk::system_Qt_widget::SlotHome(void)
{
    system.home();
}


void dvrk::system_Qt_widget::SlotArmCurrentStateEventHandler(PairStringType armState)
{
    const QString arm = armState.first;
    auto iter = m_arm_buttons.find(arm);
    QPushButton * button;
    // insert new arm if needed
    if (iter == m_arm_buttons.end()) {
        button = new QPushButton(arm);
        QVBArms->addWidget(button);
        m_arm_buttons[arm] = button;
        connect(button, &QPushButton::clicked,
                [ = ] { FocusArmButton(armState.first); });
    } else {
        button = iter->second;
    }
    // color code state
    QString state = armState.second;
    if (state == "ENABLED") {
        button->setStyleSheet("QPushButton { background-color: rgb(50, 255, 50); border: none }");
    } else if (state == "FAULT") {
        button->setStyleSheet("QPushButton { background-color: rgb(255, 100, 100); border: none }");
    } else {
        button->setStyleSheet("QPushButton { background-color: none; border: none }");
    }
}


void dvrk::system_Qt_widget::SlotSetVolume(void)
{
    double volume01 = static_cast<double>(QSVolume->value()) / 100.0;
    system.set_volume(volume01);
}


void dvrk::system_Qt_widget::setupUi(void)
{
    QHBoxLayout * mainLayout = new QHBoxLayout;

    QWidget * buttonsWidget = new QWidget();
    QVBoxLayout * boxLayout = new QVBoxLayout();
    boxLayout->setContentsMargins(0, 0, 0, 0);
    buttonsWidget->setLayout(boxLayout);

    QGroupBox * armsBox = new QGroupBox("System");
    boxLayout->addWidget(armsBox);
    QVBoxLayout * armsLayout = new QVBoxLayout();
    armsLayout->setContentsMargins(2, 2, 2, 2);
    armsBox->setLayout(armsLayout);
    QPBPowerOff = new QPushButton("Power Off");
    QPBPowerOff->setToolTip("ctrl + O");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_O), this, SLOT(SlotPowerOff()));
    armsLayout->addWidget(QPBPowerOff);
    QPBPowerOn = new QPushButton("Power On");
    QPBPowerOn->setToolTip("ctrl + P");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_P), this, SLOT(SlotPowerOn()));
    armsLayout->addWidget(QPBPowerOn);
    QPBHome = new QPushButton("Home");
    QPBHome->setToolTip("ctrl + H");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_H), this, SLOT(SlotHome()));
    armsLayout->addWidget(QPBHome);
    // arm buttons
    QVBArms = new QVBoxLayout();
    armsLayout->addLayout(QVBArms);

    QTConsoles = new QTabWidget();
    boxLayout->addWidget(QTConsoles);

    boxLayout->addStretch(100);

    QGroupBox * audioBox = new QGroupBox("Audio");
    boxLayout->addWidget(audioBox);
    QVBoxLayout * audioLayout = new QVBoxLayout();
    audioLayout->setContentsMargins(2, 2, 2, 2);
    audioBox->setLayout(audioLayout);
    QSVolume = new QSlider(Qt::Horizontal);
    QSVolume->setRange(0, 100);
    QSVolume->setValue(50);
    audioLayout->addWidget(QSVolume);

    buttonsWidget->setFixedWidth(buttonsWidget->sizeHint().width());
    mainLayout->addWidget(buttonsWidget);

    QPBComponentViewer = new QPushButton("Component\nViewer");
    QPBComponentViewer->setToolTip("Starts uDrawGraph (must be in system path)");
    boxLayout->addWidget(QPBComponentViewer);

    QLabel * labelLogo = new QLabel("");
    labelLogo->setPixmap(QPixmap(":/dVRK.png").scaled(60, 60, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    boxLayout->addWidget(labelLogo);

    QSplitter * tabWidgetAndMessages = new QSplitter();
    tabWidgetAndMessages->setOrientation(Qt::Vertical);

    QTComponents = new QTabWidget();
    tabWidgetAndMessages->addWidget(QTComponents);

    QMMessage->setupUi();
    tabWidgetAndMessages->addWidget(QMMessage);

    mainLayout->addWidget(tabWidgetAndMessages);
    setLayout(mainLayout);

    std::string title = "dVRK ";
    title.append(sawIntuitiveResearchKit_VERSION);
    title.append(" / ");
    title.append(CISST_FULL_REVISION);
    setWindowTitle(title.c_str());
    resize(sizeHint());

    // buttons
    connect(QPBPowerOff, SIGNAL(clicked()),
            this, SLOT(SlotPowerOff()));
    connect(QPBPowerOn, SIGNAL(clicked()),
            this, SLOT(SlotPowerOn()));
    connect(QPBHome, SIGNAL(clicked()),
            this, SLOT(SlotHome()));
    qRegisterMetaType<PairStringType>("PairStringType");
    connect(this, SIGNAL(SignalArmCurrentState(PairStringType)),
            this, SLOT(SlotArmCurrentStateEventHandler(PairStringType)));
    connect(QSVolume, SIGNAL(sliderReleased()),
            this, SLOT(SlotSetVolume()));
    connect(this, SIGNAL(SignalVolume(double)),
            this, SLOT(SlotVolumeEventHandler(double)));
    connect(QPBComponentViewer, SIGNAL(clicked()),
            this, SLOT(SlotComponentViewer()));

    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));
}


void dvrk::system_Qt_widget::arm_current_state_event_handler(const prmKeyValue & armState)
{
    PairStringType currentState;
    currentState.first = armState.Key.c_str();
    currentState.second = armState.Value.c_str();
    emit SignalArmCurrentState(currentState);
}


void dvrk::system_Qt_widget::SlotVolumeEventHandler(double volume)
{
    QSVolume->setValue(volume * 100.0);
}


void dvrk::system_Qt_widget::volume_event_handler(const double & volume)
{
    emit SignalVolume(volume);
}


void dvrk::system_Qt_widget::SlotComponentViewer(void)
{
    QPBComponentViewer->setEnabled(false);
    std::cerr << "Now trying to launch uDrawGraph." << std::endl
              << "uDrawGraph needs to be installed in your path and the variable UDG_HOME set." << std::endl
              << "See http://www.informatik.uni-bremen.de/uDrawGraph/en/download/download.html" << std::endl;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    mtsComponentViewer * componentViewer = new mtsComponentViewer("ComponentViewer");
    componentManager->AddComponent(componentViewer);
    osaSleep(0.2 * cmn_s);
    componentViewer->Create();
    osaSleep(0.2 * cmn_s);
    componentViewer->Start();
}


void dvrk::system_Qt_widget::FocusArmButton(const QString & armName)
{
    // determine which tab to search
    QTabWidget * subTab = QTComponents->findChild<QTabWidget *>(QString("Arms"));
    if (subTab) {
        QTComponents->setCurrentWidget(subTab);
    } else {
        subTab = QTComponents;
    }

    // now find the arm widget
    QWidget * child = subTab->findChild<QWidget *>(armName);
    if (child) {
        subTab->setCurrentWidget(child);
    } else {
        std::cerr << CMN_LOG_DETAILS << " can't find arm nor Arms tab widget for \""
                  << armName.toStdString() << "\", did you set the widget name with setObjectName?" << std::endl;
    }
}
