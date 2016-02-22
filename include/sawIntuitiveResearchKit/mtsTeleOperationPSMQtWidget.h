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

#ifndef _mtsTeleOperationPSMQtWidget_h
#define _mtsTeleOperationPSMQtWidget_h

#include <cisstVector/vctQtWidgetFrame.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <QtCore>
#include <QtGui>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

class CISST_EXPORT mtsTeleOperationPSMQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationPSMQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsTeleOperationPSMQtWidget(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    virtual void closeEvent(QCloseEvent * event);

signals:
    void SignalDesiredState(QString state);
    void SignalCurrentState(QString state);
    void SignalScale(double scale);
    void SignalRotationLocked(bool lock);
    void SignalTranslationLocked(bool lock);

    void SignalAppendMessage(QString);
    void SignalSetColor(QColor);

private slots:
    void timerEvent(QTimerEvent * event);
    void SlotTextChanged(void);
    // to set from the GUI
    void SlotSetScale(double scale);
    void SlotLockRotation(bool lock);
    void SlotLockTranslation(bool lock);
    // to update GUI from component's events
    void SlotDesiredStateEventHandler(QString state);
    void SlotCurrentStateEventHandler(QString state);
    void SlotScaleEventHandler(double scale);
    void SlotRotationLockedEventHandler(bool lock);
    void SlotTranslationLockedEventHandler(bool lock);

private:
    //! setup TeleOperationPSM controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

    void DesiredStateEventHandler(const std::string & state);
    void CurrentStateEventHandler(const std::string & state);
    void ScaleEventHandler(const double & scale);
    void RotationLockedEventHandler(const bool & lock);
    void TranslationLockedEventHandler(const bool & lock);

protected:
    struct {
        mtsFunctionWrite SetScale;
        mtsFunctionWrite LockRotation;
        mtsFunctionWrite LockTranslation;
        mtsFunctionRead GetPositionCartesianMaster;
        mtsFunctionRead GetPositionCartesianSlave;
        mtsFunctionRead GetRegistrationRotation;
        mtsFunctionRead GetPeriodStatistics;
    } TeleOperation;

private:
    QLineEdit * QLEDesiredState;
    QLineEdit * QLECurrentState;
    QCheckBox * QCBLockRotation;
    QCheckBox * QCBLockTranslation;
    QDoubleSpinBox * QSBScale;
    prmPositionCartesianGet PositionMaster;
    vctQtWidgetFrameDoubleRead * QFRPositionMasterWidget;
    prmPositionCartesianGet PositionSlave;
    vctQtWidgetFrameDoubleRead * QFRPositionSlaveWidget;
    vctMatRot3 RegistrationRotation;

    // timing
    mtsIntervalStatistics IntervalStatistics;
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;

    // messages
    void ErrorEventHandler(const std::string & message);
    void WarningEventHandler(const std::string & message);
    void StatusEventHandler(const std::string & message);
    QTextEdit * QTEMessages;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationPSMQtWidget);

#endif // _mtsTeleOperationPSMQtWidget_h
