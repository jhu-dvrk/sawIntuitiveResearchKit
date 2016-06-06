/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-17

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// cisst
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>

#include <QMessageBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QPushButton>
#include <QTextEdit>
#include <QScrollBar>
#include <QGroupBox>
#include <QTabWidget>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTime>
#include <QLabel>
#include <QPixmap>
#include <QShortcut>
#include <QDoubleSpinBox>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsoleQtWidget);

mtsIntuitiveResearchKitConsoleQtWidget::mtsIntuitiveResearchKitConsoleQtWidget(const std::string & componentName):
    mtsComponent(componentName)
{
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Main");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("PowerOff", Console.PowerOff);
        interfaceRequired->AddFunction("Home", Console.Home);
        interfaceRequired->AddFunction("TeleopEnable", Console.TeleopEnable);
        interfaceRequired->AddFunction("SetScale", Console.SetScale);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::ScaleEventHandler,
                                                this, "Scale");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::ErrorEventHandler,
                                                    this, "Error");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::WarningEventHandler,
                                                    this, "Warning");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::StatusEventHandler,
                                                    this, "Status");
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

void mtsIntuitiveResearchKitConsoleQtWidget::SlotTextChanged(void)
{
    QTEMessages->verticalScrollBar()->setSliderPosition(QTEMessages->verticalScrollBar()->maximum());
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

    boxLayout->addStretch(100);
    buttonsWidget->setFixedWidth(buttonsWidget->sizeHint().width());
    mainLayout->addWidget(buttonsWidget);

    QLabel * labelLogo = new QLabel("");
    labelLogo->setPixmap(QPixmap(":/dVRK.svg"));
    boxLayout->addWidget(labelLogo);

    QSplitter * tabWidgetAndMessages = new QSplitter();
    tabWidgetAndMessages->setOrientation(Qt::Vertical);

    QTWidgets = new QTabWidget();
    tabWidgetAndMessages->addWidget(QTWidgets);

    QTEMessages = new QTextEdit();
    QTEMessages->setReadOnly(true);
    QTEMessages->ensureCursorVisible();
    QTEMessages->resize(QTEMessages->width(), 600);
    tabWidgetAndMessages->addWidget(QTEMessages);

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

    // messages
    connect(this, SIGNAL(SignalAppendMessage(QString)),
            QTEMessages, SLOT(append(QString)));
    connect(this, SIGNAL(SignalSetColor(QColor)),
            QTEMessages, SLOT(setTextColor(QColor)));
    connect(QTEMessages, SIGNAL(textChanged()),
            this, SLOT(SlotTextChanged()));

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

void mtsIntuitiveResearchKitConsoleQtWidget::ErrorEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("red"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Error: ") + QString(message.c_str()));
}

void mtsIntuitiveResearchKitConsoleQtWidget::WarningEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("darkRed"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Warning: ") + QString(message.c_str()));
}

void mtsIntuitiveResearchKitConsoleQtWidget::StatusEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("black"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Status: ") + QString(message.c_str()));
}
