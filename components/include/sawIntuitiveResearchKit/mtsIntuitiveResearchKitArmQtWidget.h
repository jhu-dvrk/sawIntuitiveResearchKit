/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-08-24

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitArmQtWidget_h
#define _mtsIntuitiveResearchKitArmQtWidget_h

#include <cisstVector/vctForceTorqueQtWidget.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>

#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmStateJointQtWidget.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianGetQtWidget.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmOperatingStateQtWidget.h>

#include <QWidget>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

class QCheckBox;
class QPushButton;
class QTextEdit;

class CISST_EXPORT mtsIntuitiveResearchKitArmQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitArmQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsIntuitiveResearchKitArmQtWidget() {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    virtual void closeEvent(QCloseEvent * event);

signals:
    void SignalDesiredState(QString state);
    void SignalCurrentState(QString state);
    void SignalOperatingState(const prmOperatingState & state);

private slots:
    void timerEvent(QTimerEvent * event);
    void SlotDesiredStateEventHandler(QString state);
    void SlotCurrentStateEventHandler(QString state);
    void SlotOperatingStateEventHandler(const prmOperatingState & state);
    void SlotLogEnabled(void);
    void SlotEnableDirectControl(bool toggle);
    void SlotHome(void);

private:
    //! setup GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

protected:
    struct ArmStruct {
        mtsFunctionRead GetStateJoint;
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionRead GetWrenchBody;
        mtsFunctionWrite SetDesiredState;
        mtsFunctionRead GetPeriodStatistics;
    } Arm;

    // so derived class has access to custom parts of widget
    QVBoxLayout * MainLayout;
    mtsInterfaceRequired * InterfaceRequired;
    inline virtual void setupUiDerived(void) {};
    inline virtual void timerEventDerived(void) {};

private:
    bool DirectControl;

    prmStateJoint StateJoint;
    prmStateJointQtWidget * QSJWidget;

    prmPositionCartesianGet Position;
    prmPositionCartesianGetQtWidget * QCPGWidget;

    prmForceCartesianGet Wrench;
    vctForceTorqueQtWidget * QFTWidget;

    // timing
    mtsIntervalStatistics IntervalStatistics;
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;

    // state
    QCheckBox * QCBEnableDirectControl;
    QPushButton * QPBHome;

    QLineEdit * QLEDesiredState;
    QLineEdit * QLECurrentState;
    void DesiredStateEventHandler(const std::string & state);
    void CurrentStateEventHandler(const std::string & state);

    prmOperatingStateQtWidget * QPOState;
    void OperatingStateEventHandler(const prmOperatingState & state);

    // messages
    bool LogEnabled;
    QPushButton * QPBLog;
    mtsMessageQtWidget * QMMessage;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitArmQtWidget);

#endif // _mtsIntuitiveResearchKitArmQtWidget_h
