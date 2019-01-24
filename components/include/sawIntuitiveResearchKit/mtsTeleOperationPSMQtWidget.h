/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsTeleOperationPSMQtWidget_h
#define _mtsTeleOperationPSMQtWidget_h

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianGetQtWidget.h>

#include <QSplitter>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

class QCheckBox;
class QDoubleSpinBox;
class QPushButton;
class QTextEdit;

class CISST_EXPORT mtsTeleOperationPSMQtWidget: public QSplitter, public mtsComponent
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
    void SignalFollowing(bool following);
    void SignalScale(double scale);
    void SignalRotationLocked(bool lock);
    void SignalTranslationLocked(bool lock);

private slots:
    void timerEvent(QTimerEvent * event);
    void SlotLogEnabled(void);
    // to set from the GUI
    void SlotSetScale(double scale);
    void SlotLockRotation(bool lock);
    void SlotLockTranslation(bool lock);
    // to update GUI from component's events
    void SlotDesiredStateEventHandler(QString state);
    void SlotCurrentStateEventHandler(QString state);
    void SlotFollowingEventHandler(bool following);
    void SlotScaleEventHandler(double scale);
    void SlotRotationLockedEventHandler(bool lock);
    void SlotTranslationLockedEventHandler(bool lock);

private:
    //! setup TeleOperationPSM controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

    void DesiredStateEventHandler(const std::string & state);
    void CurrentStateEventHandler(const std::string & state);
    void FollowingEventHandler(const bool & following);
    void ScaleEventHandler(const double & scale);
    void RotationLockedEventHandler(const bool & lock);
    void TranslationLockedEventHandler(const bool & lock);

protected:
    struct {
        mtsFunctionWrite SetScale;
        mtsFunctionWrite LockRotation;
        mtsFunctionWrite LockTranslation;
        mtsFunctionRead GetPositionCartesianMTM;
        mtsFunctionRead GetPositionCartesianPSM;
        mtsFunctionRead GetRegistrationRotation;
        mtsFunctionRead GetPeriodStatistics;
    } TeleOperation;

private:
    QLineEdit * QLEDesiredState;
    QLineEdit * QLECurrentState;
    QLineEdit * QLEFollowing;
    QCheckBox * QCBLockRotation;
    QCheckBox * QCBLockTranslation;
    QDoubleSpinBox * QSBScale;
    prmPositionCartesianGet PositionMTM;
    prmPositionCartesianGetQtWidget * QCPGMTMWidget;
    prmPositionCartesianGet PositionPSM;
    prmPositionCartesianGetQtWidget * QCPGPSMWidget;
    vctMatRot3 RegistrationRotation;

    // timing
    mtsIntervalStatistics IntervalStatistics;
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;

    // messages
    bool LogEnabled;
    QPushButton * QPBLog;
    mtsMessageQtWidget * QMMessage;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationPSMQtWidget);

#endif // _mtsTeleOperationPSMQtWidget_h
