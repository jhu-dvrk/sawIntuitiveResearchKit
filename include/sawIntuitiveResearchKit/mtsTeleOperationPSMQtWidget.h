/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsTeleOperationQtWidget_h
#define _mtsTeleOperationQtWidget_h

#include <cisstVector/vctQtWidgetFrame.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <QtCore>
#include <QtGui>

#include <sawControllers/sawControllersQtExport.h>

class CISST_EXPORT mtsTeleOperationQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsTeleOperationQtWidget(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    virtual void closeEvent(QCloseEvent * event);

signals:
    void SignalEnable(bool enable);
    void SignalScale(double scale);
    void SignalRotationLocked(bool lock);
    void SignalTranslationLocked(bool lock);

    void SignalAppendMessage(QString);
    void SignalSetColor(QColor);

private slots:
    void timerEvent(QTimerEvent * event);
    void SlotTextChanged(void);
    // to set from the GUI
    void SlotEnable(bool state);
    void SlotSetScale(double scale);
    void SlotLockRotation(bool lock);
    void SlotLockTranslation(bool lock);
    // to update GUI from component's events
    void SlotEnableEventHandler(bool state);
    void SlotScaleEventHandler(double scale);
    void SlotRotationLockedEventHandler(bool lock);
    void SlotTranslationLockedEventHandler(bool lock);

private:
    //! setup TeleOperation controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

    void EnableEventHandler(const bool & enable);
    void ScaleEventHandler(const double & scale);
    void RotationLockedEventHandler(const bool & lock);
    void TranslationLockedEventHandler(const bool & lock);

protected:
    struct {
        mtsFunctionWrite Enable;
        mtsFunctionWrite SetScale;
        mtsFunctionWrite LockRotation;
        mtsFunctionWrite LockTranslation;
        mtsFunctionRead GetPositionCartesianMaster;
        mtsFunctionRead GetPositionCartesianSlave;
        mtsFunctionRead GetRegistrationRotation;
        mtsFunctionRead GetPeriodStatistics;
    } TeleOperation;

private:
    QCheckBox * QCBEnable;
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

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationQtWidget);

#endif // _mtsTeleOperationQtWidget_h
