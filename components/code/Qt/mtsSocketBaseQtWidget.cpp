/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-08-24

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// Qt include
#include <QString>
#include <QtGui>
#include <QMessageBox>
// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawIntuitiveResearchKit/mtsSocketBaseQtWidget.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsSocketBaseQtWidget, mtsComponent, std::string);

mtsSocketBaseQtWidget::mtsSocketBaseQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000)
{

    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("SocketBase");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPacketsLost", SocketBase.GetPacketsLost);
        interfaceRequired->AddFunction("GetPacketsDelayed", SocketBase.GetPacketsDelayed);
        interfaceRequired->AddFunction("GetLoopTime", SocketBase.GetLoopTime);
        interfaceRequired->AddFunction("GetLastReceivedPacketId", SocketBase.GetLastReceivedPacketId);
        interfaceRequired->AddFunction("GetLastSentPacketId", SocketBase.GetLastSentPacketId);
        interfaceRequired->AddFunction("GetPeriodStatistics", SocketBase.GetPeriodStatistics);
    }
}

void mtsSocketBaseQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsSocketBaseQtWidget::Startup(void)
{
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms    
    if (!parent()) {
        show();
    }
}

void mtsSocketBaseQtWidget::Cleanup(void)
{
    this->hide();
}

void mtsSocketBaseQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsSocketBaseQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsSocketBaseQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    unsigned int packet;
    SocketBase.GetLastSentPacketId(packet);
    SocketBase.QLLastSentPacketId->setText(QString::number(packet));

    SocketBase.GetLastReceivedPacketId(packet);
    SocketBase.QLLastReceivedPacketId->setText(QString::number(packet));

    SocketBase.GetPacketsLost(packet);
    SocketBase.QLPacketsLost->setText(QString::number(packet));

    SocketBase.GetPacketsDelayed(packet);
    SocketBase.QLPacketsDelayed->setText(QString::number(packet));

    double loopTime;
    SocketBase.GetLoopTime(loopTime);
    SocketBase.QLLoopTime->setText(QString::number(loopTime * 1000.0, 'g', 3));

    SocketBase.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsSocketBaseQtWidget::setupUi(void)
{
    QHBoxLayout * mainLayout = new QHBoxLayout;
    setLayout(mainLayout);
    setWindowTitle("Manipulator");
    resize(sizeHint());

    // Socket statistics
    QVBoxLayout * socketlayout = new QVBoxLayout();
    QGridLayout * grid = new QGridLayout();
    socketlayout->addLayout(grid);
    mainLayout->addLayout(socketlayout);

    int row = 0;
    grid->addWidget(new QLabel("Sent Id"), row, 0);
    SocketBase.QLLastSentPacketId = new QLabel();
    grid->addWidget(SocketBase.QLLastSentPacketId, row, 1);
    row++;

    grid->addWidget(new QLabel("Received Id"), row, 0);
    SocketBase.QLLastReceivedPacketId = new QLabel();
    grid->addWidget(SocketBase.QLLastReceivedPacketId, row, 1);
    row++;

    grid->addWidget(new QLabel("Packets lost"), row, 0);
    SocketBase.QLPacketsLost = new QLabel();
    grid->addWidget(SocketBase.QLPacketsLost, row, 1);
    row++;

    grid->addWidget(new QLabel("Packets delayed"), row, 0);
    SocketBase.QLPacketsDelayed = new QLabel();
    grid->addWidget(SocketBase.QLPacketsDelayed, row, 1);
    row++;

    grid->addWidget(new QLabel("Loop time (ms)"), row, 0);
    SocketBase.QLLoopTime = new QLabel();
    grid->addWidget(SocketBase.QLLoopTime, row, 1);
    row++;
    socketlayout->addStretch();

    // timing
    QVBoxLayout * timingLayout = new QVBoxLayout();
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    timingLayout->addWidget(QMIntervalStatistics);
    timingLayout->addStretch();
    mainLayout->addLayout(timingLayout);
}
