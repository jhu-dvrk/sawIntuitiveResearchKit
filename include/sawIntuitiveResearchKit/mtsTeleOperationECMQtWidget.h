/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsTeleOperationECMQtWidget_h
#define _mtsTeleOperationECMQtWidget_h

#include <cisstVector/vctQtWidgetFrame.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <QtCore>
#include <QtGui>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsTeleOperationECMQtWidget: public QSplitter, public mtsComponent
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
    void SignalScale(double scale);

    void SignalAppendMessage(QString);
    void SignalSetColor(QColor);

private slots:
    void timerEvent(QTimerEvent * event);
    void SlotTextChanged(void);
    void SlotLogEnabled(void);
    // to set from the GUI
    void SlotDesiredStateEventHandler(QString state);
    void SlotCurrentStateEventHandler(QString state);
    void SlotSetScale(double scale);
    // to update GUI from component's events
    void SlotScaleEventHandler(double scale);

private:
    //! setup TeleOperationECM controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

    void DesiredStateEventHandler(const std::string & state);
    void CurrentStateEventHandler(const std::string & state);
    void ScaleEventHandler(const double & scale);

protected:
    struct {
        mtsFunctionWrite SetScale;
        mtsFunctionRead GetPositionCartesianMasterLeft;
        mtsFunctionRead GetPositionCartesianMasterRight;
        mtsFunctionRead GetPositionCartesianSlave;
        mtsFunctionRead GetRegistrationRotation;
        mtsFunctionRead GetPeriodStatistics;
    } TeleOperation;

private:
    QLineEdit * QLEDesiredState;
    QLineEdit * QLECurrentState;
    QDoubleSpinBox * QSBScale;
    prmPositionCartesianGet PositionMasterLeft;
    vctQtWidgetFrameDoubleRead * QFRPositionMasterLeftWidget;
    prmPositionCartesianGet PositionMasterRight;
    vctQtWidgetFrameDoubleRead * QFRPositionMasterRightWidget;
    prmPositionCartesianGet PositionSlave;
    vctQtWidgetFrameDoubleRead * QFRPositionSlaveWidget;
    vctMatRot3 RegistrationRotation;

    // timing
    mtsIntervalStatistics IntervalStatistics;
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;

    // messages
    bool LogEnabled;
    QPushButton * QPBLog;
    void ErrorEventHandler(const std::string & message);
    void WarningEventHandler(const std::string & message);
    void StatusEventHandler(const std::string & message);
    QTextEdit * QTEMessages;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationECMQtWidget);

#endif // _mtsTeleOperationECMQtWidget_h
