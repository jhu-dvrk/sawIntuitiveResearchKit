/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsTeleOperationECMQtWidget_h
#define _mtsTeleOperationECMQtWidget_h

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>
#include <cisstMultiTask/mtsIntervalStatisticsQtWidget.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianGetQtWidget.h>

#include <QSplitter>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

class QDoubleSpinBox;
class QPushButton;
class QTextEdit;

class CISST_EXPORT mtsTeleOperationECMQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationECMQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsTeleOperationECMQtWidget(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    virtual void closeEvent(QCloseEvent * event);

signals:
    void SignalDesiredState(QString state);
    void SignalCurrentState(QString state);
    void SignalFollowing(bool following);
    void SignalScale(double scale);

private slots:
    void timerEvent(QTimerEvent * event);
    void SlotLogEnabled(void);
    // to set from the GUI
    void SlotDesiredStateEventHandler(QString state);
    void SlotCurrentStateEventHandler(QString state);
    void SlotSetScale(double scale);
    // to update GUI from component's events
    void SlotFollowingEventHandler(bool following);
    void SlotScaleEventHandler(double scale);

private:
    //! setup TeleOperationECM controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

    void DesiredStateEventHandler(const std::string & state);
    void CurrentStateEventHandler(const std::string & state);
    void FollowingEventHandler(const bool & following);
    void ScaleEventHandler(const double & scale);

protected:
    struct {
        mtsFunctionWrite set_scale;
        mtsFunctionRead MTML_measured_cp;
        mtsFunctionRead MTMR_measured_cp;
        mtsFunctionRead ECM_measured_cp;
        mtsFunctionRead period_statistics;
    } TeleOperation;

private:
    QLineEdit * QLEDesiredState;
    QLineEdit * QLECurrentState;
    QLineEdit * QLEFollowing;
    QDoubleSpinBox * QSBScale;
    prmPositionCartesianGet m_MTML_measured_cp;
    prmPositionCartesianGetQtWidget * QCPGMTMLWidget;
    prmPositionCartesianGet m_MTMR_measured_cp;
    prmPositionCartesianGetQtWidget * QCPGMTMRWidget;
    prmPositionCartesianGet m_ECM_measured_cp;
    prmPositionCartesianGetQtWidget * QCPGECMWidget;

    // timing
    mtsIntervalStatistics m_interval_statistics;
    mtsIntervalStatisticsQtWidget * QMIntervalStatistics;

    // messages
    bool LogEnabled;
    QPushButton * QPBLog;
    mtsMessageQtWidget * QMMessage;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationECMQtWidget);

#endif // _mtsTeleOperationECMQtWidget_h
