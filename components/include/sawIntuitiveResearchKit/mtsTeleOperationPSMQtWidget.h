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
    void SignalFollowing(bool following);
    void SignalScale(double scale);
    void SignalRotationLocked(bool lock);
    void SignalTranslationLocked(bool lock);
    void SignalAlignMTM(bool align);

private slots:
    void timerEvent(QTimerEvent * event);
    void SlotLogEnabled(void);
    // to set from the GUI
    void SlotSetScale(double scale);
    void SlotLockRotation(bool lock);
    void SlotLockTranslation(bool lock);
    void SlotSetAlignMTM(bool align);
    // to update GUI from component's events
    void SlotDesiredStateEventHandler(QString state);
    void SlotCurrentStateEventHandler(QString state);
    void SlotFollowingEventHandler(bool following);
    void SlotScaleEventHandler(double scale);
    void SlotRotationLockedEventHandler(bool lock);
    void SlotTranslationLockedEventHandler(bool lock);
    void SlotAlignMTMEventHandler(bool align);

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
    void AlignMTMEventHandler(const bool & align);

protected:
    struct {
        mtsFunctionWrite set_scale;
        mtsFunctionWrite lock_rotation;
        mtsFunctionWrite lock_translation;
        mtsFunctionWrite set_align_mtm;
        mtsFunctionRead MTM_measured_cp;
        mtsFunctionRead PSM_setpoint_cp;
        mtsFunctionRead align_mtm;
        mtsFunctionRead alignment_offset;
        mtsFunctionRead registration_rotation;
        mtsFunctionRead period_statistics;
    } TeleOperation;

private:
    QLineEdit * QLEDesiredState;
    QLineEdit * QLECurrentState;
    QLineEdit * QLEFollowing;
    QCheckBox * QCBLockRotation;
    QCheckBox * QCBLockTranslation;
    QCheckBox * QCBAlignMTM;
    QDoubleSpinBox * QSBScale;
    prmPositionCartesianGet m_MTM_measured_cp;
    prmPositionCartesianGetQtWidget * QCPGMTMWidget;
    prmPositionCartesianGet m_PSM_setpoint_cp;
    prmPositionCartesianGetQtWidget * QCPGPSMWidget;
    vctMatRot3 m_alignment_offset;
    vctQtWidgetRotationDoubleRead * QVRAlignOffset;
    vctMatRot3 m_registration_rotation;

    // timing
    mtsIntervalStatistics m_interval_statistics;
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;

    // messages
    bool LogEnabled;
    QPushButton * QPBLog;
    mtsMessageQtWidget * QMMessage;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationPSMQtWidget);

#endif // _mtsTeleOperationPSMQtWidget_h
