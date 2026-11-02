/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Youri Tan
  Created on: 2013-08-24

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitSUJQtWidget_h
#define _mtsIntuitiveResearchKitSUJQtWidget_h

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>


class CISST_EXPORT mtsIntuitiveResearchKitSUJQtWidget: public mtsIntuitiveResearchKitArmQtWidget
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitSUJQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsIntuitiveResearchKitSUJQtWidget() {}

    void Startup(void) override;

protected:
    void setupUiDerived(void) override;
    void timerEventDerived(void) override;

    vctQtWidgetDynamicVectorDoubleRead * QVBrakeCurrentWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVExtraVoltagesWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVPrimaryVoltagesWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVSecondaryVoltagesWidget;

    vctQtWidgetDynamicVectorDoubleWrite * QVPotentiometerRecalibrationStartWidget;
    vctQtWidgetDynamicVectorDoubleWrite * QVPotentiometerRecalibrationFinishWidget;

    bool mShowMore;

    QPushButton * QPBClutch;
    QPushButton * QPBShowMore;
    QWidget * QWMore;
    QPushButton * QPBLiftDown;
    QPushButton * QPBLiftUp;
    QPushButton * QPBManualRecalibration;

    mtsFunctionWrite Clutch;
    mtsFunctionWrite SetLiftVelocity;

    Eigen::VectorXd JointPositionStart;
    Eigen::VectorXd JointPositionFinish;
    Eigen::VectorXd JointVoltageStart;
    Eigen::VectorXd JointVoltageFinish;
    double BrakeCurrent;

    mtsFunctionRead GetBrakeCurrent;

    mtsFunctionRead GetPrimaryVoltages;
    mtsFunctionRead GetSecondaryVoltages;
    mtsFunctionRead GetExtraVoltages;
    Eigen::MatrixXd mJointsRecalibrationMatrix;
    Eigen::VectorXd mVoltages[2];
    Eigen::VectorXd mVoltagesExtra;
    mtsFunctionWrite SetRecalibrationMatrix;

protected slots:
    void SlotShowMore(void);

    void SlotManualRecalibration(void);
    void SlotClutchPressed(void);
    void SlotClutchReleased(void);

    void SlotVelocityDownPressed(void);
    void SlotVelocityUpPressed(void);
    void SlotVelocityReleased(void);

    void SlotRecalibrationStartChanged(void);
    void SlotRecalibrationFinishChanged(void);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitSUJQtWidget);

#endif // _mtsIntuitiveResearchKitSUJQtWidget_h
