/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Pretham Chalasani, Anton Deguet
  Created on: 2016-11-04

  (C) Copyright 2016-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsSocketBaseQtWidget_h
#define _mtsSocketBaseQtWidget_h

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>

class mtsSocketBaseQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsSocketBaseQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsSocketBaseQtWidget() {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    virtual void closeEvent(QCloseEvent * event);

private slots:
    void timerEvent(QTimerEvent * event);

private:
    //! setup GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

protected:
    struct {
        mtsFunctionRead GetPacketsLost;
        mtsFunctionRead GetPacketsDelayed;
        mtsFunctionRead GetLoopTime;
        mtsFunctionRead GetLastReceivedPacketId;
        mtsFunctionRead GetLastSentPacketId;
        mtsFunctionRead GetPeriodStatistics;

        QLabel * QLPacketsLost;
        QLabel * QLPacketsDelayed;
        QLabel * QLLoopTime;
        QLabel * QLLastReceivedPacketId;
        QLabel * QLLastSentPacketId;
    } SocketBase;

private:

    // timing
    mtsIntervalStatistics IntervalStatistics;
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSocketBaseQtWidget);

#endif // _mtsSocketBaseQtWidget_h
