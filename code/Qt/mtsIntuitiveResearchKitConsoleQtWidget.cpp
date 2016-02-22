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
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>

#include <QMessageBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QPushButton>
#include <QTextEdit>
#include <QScrollBar>
#include <QTabWidget>
#include <QSplitter>
// #include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTime>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsoleQtWidget);

mtsIntuitiveResearchKitConsoleQtWidget::mtsIntuitiveResearchKitConsoleQtWidget(const std::string & componentName):
    mtsComponent(componentName)
{
    mtsInterfaceRequired * interfaceRequiredMain = AddInterfaceRequired("Main");
    if (interfaceRequiredMain) {
        interfaceRequiredMain->AddFunction("PowerOff", Console.PowerOff);
        interfaceRequiredMain->AddFunction("Home", Console.Home);
        interfaceRequiredMain->AddFunction("TeleopEnable", Console.TeleopEnable);
        interfaceRequiredMain->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::ErrorEventHandler,
                                                    this, "Error");
        interfaceRequiredMain->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::WarningEventHandler,
                                                    this, "Warning");
        interfaceRequiredMain->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::StatusEventHandler,
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

void mtsIntuitiveResearchKitConsoleQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsIntuitiveResearchKitConsoleQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
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

void mtsIntuitiveResearchKitConsoleQtWidget::SlotTeleop(void)
{
    Console.TeleopEnable(QPBTeleop->isChecked());
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotTextChanged(void)
{
    QTEMessages->verticalScrollBar()->setSliderPosition(QTEMessages->verticalScrollBar()->maximum());
}

void mtsIntuitiveResearchKitConsoleQtWidget::setupUi(void)
{
    QHBoxLayout * mainLayout = new QHBoxLayout;

    QPBPowerOff = new QPushButton("Idle");
    QPBHome = new QPushButton("Home");
    QPBTeleop = new QPushButton("Teleop");
    QPBTeleop->setCheckable(true);

    QVBoxLayout * buttonsLayout = new QVBoxLayout;
    buttonsLayout->addWidget(QPBPowerOff);
    buttonsLayout->addWidget(QPBHome);
    buttonsLayout->addWidget(QPBTeleop);
    buttonsLayout->addStretch(1);
    mainLayout->addLayout(buttonsLayout);

    QSplitter * tabWidgetAndMessages = new QSplitter(); 
    tabWidgetAndMessages->setOrientation(Qt::Vertical);
    
    QTWidgets = new QTabWidget();
    tabWidgetAndMessages->addWidget(QTWidgets);

    QTEMessages = new QTextEdit();
    QTEMessages->setReadOnly(true);
    QTEMessages->ensureCursorVisible();
    tabWidgetAndMessages->addWidget(QTEMessages);

    mainLayout->addWidget(tabWidgetAndMessages);

    // QVBoxLayout * mainLayout = new QVBoxLayout;
    // mainLayout->addLayout(frameLayout);
    setLayout(mainLayout);

    setWindowTitle("Intuitive Research Kit");
    resize(sizeHint());

    // buttons
    connect(QPBPowerOff, SIGNAL(clicked()),
            this, SLOT(SlotPowerOff()));
    connect(QPBHome, SIGNAL(clicked()),
            this, SLOT(SlotHome()));
    connect(QPBTeleop, SIGNAL(clicked()),
            this, SLOT(SlotTeleop()));

    // messages
    connect(this, SIGNAL(SignalAppendMessage(QString)),
            QTEMessages, SLOT(append(QString)));
    connect(this, SIGNAL(SignalSetColor(QColor)),
            QTEMessages, SLOT(setTextColor(QColor)));
    connect(QTEMessages, SIGNAL(textChanged()),
            this, SLOT(SlotTextChanged()));
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
