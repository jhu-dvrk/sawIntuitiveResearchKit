/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-17

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsComponentViewer.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>

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
#include <QRadioButton>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsoleQtWidget);

mtsIntuitiveResearchKitConsoleQtWidget::mtsIntuitiveResearchKitConsoleQtWidget(const std::string & componentName):
    mtsComponent(componentName)
{
    QMMessage = new mtsMessageQtWidget();

    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Main");
    if (interfaceRequired) {
        QMMessage->SetInterfaceRequired(interfaceRequired);
        interfaceRequired->AddFunction("PowerOff", Console.PowerOff);
        interfaceRequired->AddFunction("Home", Console.Home);
        interfaceRequired->AddFunction("TeleopEnable", Console.TeleopEnable);
        interfaceRequired->AddFunction("SetScale", Console.SetScale);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::ScaleEventHandler,
                                                this, "Scale");
    }
    interfaceRequired = AddInterfaceRequired("Clutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::ClutchEventHandler,
                                                this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("OperatorPresent");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::OperatorPresentEventHandler,
                                                this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("Camera");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::CameraEventHandler,
                                                this, "Button");
    }
    setupUi();
}

void mtsIntuitiveResearchKitConsoleQtWidget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsIntuitiveResearchKitConsoleQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveResearchKitConsoleQtWidget::Startup" << std::endl;
    if (!parent()) {
        show();
    }

        // write warning to cerr if not compiled in Release mode
    if (std::string(CISST_BUILD_TYPE) != "Release") {
        std::string message;
        message.append("Warning:\n");
        message.append(" It seems that \"cisst\" has not been compiled in\n");
        message.append(" Release mode.  Make sure your CMake configuration\n");
        message.append(" or catkin profile is configured to compile in\n");
        message.append(" Release mode for better performance and stability");

        QMessageBox * msgBox = new QMessageBox(this);
        msgBox->setAttribute(Qt::WA_DeleteOnClose); // makes sure the msgbox is deleted automatically when closed
        msgBox->setStandardButtons(QMessageBox::Ok);
        msgBox->setWindowTitle("Warning");
        msgBox->setText(message.c_str());
        msgBox->setModal(false); // if you want it non-modal
        msgBox->show();
    }
}

void mtsIntuitiveResearchKitConsoleQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveResearchKitConsoleQtWidget::Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitConsoleQtWidget::HasTeleOp(const bool & hasTeleOp)
{
    QPBTeleopStart->setEnabled(hasTeleOp);
    QPBTeleopStop->setEnabled(hasTeleOp);
    QSBScale->setEnabled(hasTeleOp);
}

void mtsIntuitiveResearchKitConsoleQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsIntuitiveResearchKitConsoleQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        this->hide();
        // send clean power off message and wait a bit
        Console.PowerOff();
        osaSleep(2.0 * cmn_s);
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotPowerOff(void)
{
    Console.PowerOff();
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotHome(void)
{
    Console.Home();
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotTeleopStart(void)
{
    Console.TeleopEnable(true);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotTeleopStop(void)
{
    Console.TeleopEnable(false);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotSetScale(double scale)
{
    Console.SetScale(scale);
}

void mtsIntuitiveResearchKitConsoleQtWidget::setupUi(void)
{
    QHBoxLayout * mainLayout = new QHBoxLayout;

    QWidget * buttonsWidget = new QWidget();
    QVBoxLayout * boxLayout = new QVBoxLayout();
    boxLayout->setContentsMargins(0, 0, 0, 0);
    buttonsWidget->setLayout(boxLayout);

    QGroupBox * powerBox = new QGroupBox("Power");
    boxLayout->addWidget(powerBox);
    QVBoxLayout * powerLayout = new QVBoxLayout();
    powerBox->setLayout(powerLayout);
    QPBPowerOff = new QPushButton("Off");
    QPBPowerOff->setToolTip("ctrl + O");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_O), this, SLOT(SlotPowerOff()));
    powerLayout->addWidget(QPBPowerOff);
    QPBHome = new QPushButton("Home");
    QPBHome->setToolTip("ctrl + H");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_H), this, SLOT(SlotHome()));
    powerLayout->addWidget(QPBHome);

    QGroupBox * teleopBox = new QGroupBox("Teleop");
    boxLayout->addWidget(teleopBox);
    QVBoxLayout * teleopLayout = new QVBoxLayout();
    teleopBox->setLayout(teleopLayout);
    QPBTeleopStart = new QPushButton("Start");
    QPBTeleopStart->setToolTip("ctrl + T");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_T), this, SLOT(SlotTeleopStart()));
    teleopLayout->addWidget(QPBTeleopStart);
    QPBTeleopStop = new QPushButton("Stop");
    QPBTeleopStop->setToolTip("ctrl + S");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_S), this, SLOT(SlotTeleopStop()));
    teleopLayout->addWidget(QPBTeleopStop);
    QSBScale = new QDoubleSpinBox();
    QSBScale->setRange(0.1, 1.0);
    QSBScale->setSingleStep(0.1);
    QSBScale->setPrefix("scale ");
    QSBScale->setValue(0.2);
    teleopLayout->addWidget(QSBScale);

    QGroupBox * inputsBox = new QGroupBox("Inputs");
    boxLayout->addWidget(inputsBox);
    QVBoxLayout * inputsLayout = new QVBoxLayout();
    inputsBox->setLayout(inputsLayout);
    QRBClutch = new QRadioButton("Clutch");
    QRBClutch->setAutoExclusive(false);
    QRBClutch->setChecked(false);
    QRBClutch->setEnabled(false);
    inputsLayout->addWidget(QRBClutch);
    QRBOperatorPresent = new QRadioButton("Operator");
    QRBOperatorPresent->setAutoExclusive(false);
    QRBOperatorPresent->setChecked(false);
    QRBOperatorPresent->setEnabled(false);
    inputsLayout->addWidget(QRBOperatorPresent);
    QRBCamera = new QRadioButton("Camera");
    QRBCamera->setAutoExclusive(false);
    QRBCamera->setChecked(false);
    QRBCamera->setEnabled(false);
    inputsLayout->addWidget(QRBCamera);

    boxLayout->addStretch(100);
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

    QTWidgets = new QTabWidget();
    tabWidgetAndMessages->addWidget(QTWidgets);

    QMMessage->setupUi();
    tabWidgetAndMessages->addWidget(QMMessage);

    mainLayout->addWidget(tabWidgetAndMessages);
    setLayout(mainLayout);

    std::string title = "dVRK ";
    title.append(sawIntuitiveResearchKit_VERSION);
    title.append(" / cisst ");
    title.append(CISST_VERSION);
    setWindowTitle(title.c_str());
    resize(sizeHint());

    // buttons
    connect(QPBPowerOff, SIGNAL(clicked()),
            this, SLOT(SlotPowerOff()));
    connect(QPBHome, SIGNAL(clicked()),
            this, SLOT(SlotHome()));
    connect(QPBTeleopStart, SIGNAL(clicked()),
            this, SLOT(SlotTeleopStart()));
    connect(QPBTeleopStop, SIGNAL(clicked()),
            this, SLOT(SlotTeleopStop()));
    connect(QSBScale, SIGNAL(valueChanged(double)),
            this, SLOT(SlotSetScale(double)));
    connect(this, SIGNAL(SignalScale(double)),
            this, SLOT(SlotScaleEventHandler(double)));
    connect(this, SIGNAL(SignalClutch(bool)),
            this, SLOT(SlotClutchEventHandler(bool)));
    connect(this, SIGNAL(SignalOperatorPresent(bool)),
            this, SLOT(SlotOperatorPresentEventHandler(bool)));
    connect(this, SIGNAL(SignalCamera(bool)),
            this, SLOT(SlotCameraEventHandler(bool)));
    connect(QPBComponentViewer, SIGNAL(clicked()),
            this, SLOT(SlotComponentViewer()));

    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotScaleEventHandler(double scale)
{
    QSBScale->setValue(scale);
}

void mtsIntuitiveResearchKitConsoleQtWidget::ScaleEventHandler(const double & scale)
{
    emit SignalScale(scale);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotClutchEventHandler(bool clutch)
{
    QRBClutch->setChecked(clutch);
}

void mtsIntuitiveResearchKitConsoleQtWidget::ClutchEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        emit SignalClutch(true);
    } else {
        emit SignalClutch(false);
    }
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotOperatorPresentEventHandler(bool operatorPresent)
{
    QRBOperatorPresent->setChecked(operatorPresent);
}

void mtsIntuitiveResearchKitConsoleQtWidget::OperatorPresentEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        emit SignalOperatorPresent(true);
    } else {
        emit SignalOperatorPresent(false);
    }
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotCameraEventHandler(bool camera)
{
    QRBCamera->setChecked(camera);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotComponentViewer(void)
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

void mtsIntuitiveResearchKitConsoleQtWidget::CameraEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        emit SignalCamera(true);
    } else {
        emit SignalCamera(false);
    }

}
