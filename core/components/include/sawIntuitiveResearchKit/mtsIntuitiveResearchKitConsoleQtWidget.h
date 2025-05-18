/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitConsoleQtWidget_h
#define _mtsIntuitiveResearchKitConsoleQtWidget_h

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmKeyValue.h>

class QPushButton;
class QRadioButton;
class QTabWidget;
class QDoubleSpinBox;
class QSlider;
class QVBoxLayout;
class QCheckBox;

#include <QWidget>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

class CISST_EXPORT mtsIntuitiveResearchKitConsoleQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

    typedef QPair<QString, QString> PairStringType;
public:
    mtsIntuitiveResearchKitConsoleQtWidget(const std::string & componentName);
    inline virtual ~mtsIntuitiveResearchKitConsoleQtWidget() {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);
    inline QTabWidget * GetTabWidget(void) {
        return QTWidgets;
    }
    void HasTeleOp(const bool & hasTeleOp);

signals:
    void SignalArmCurrentState(PairStringType armState);
    void SignalTeleopPSMSelected(PairStringType selected);
    void SignalTeleopPSMUnselected(PairStringType unselected);
    void SignalTeleopEnabled(bool toggle);
    void SignalScale(double scale);
    void SignalOperatorPresent(bool operatorPresent);
    void SignalClutch(bool clutch);
    void SignalCamera(bool camera);
    void SignalVolume(double volume);

protected:
    void FocusArmButton(const QString & armName);
    void GetTeleopButtonCheck(const PairStringType & pair, QPushButton * & button, QCheckBox * & check);
    void FocusTeleopButton(const QString & teleop);
    void SelectTeleopCheck(const PairStringType & pair);
    void UnselectTeleopCheck(const PairStringType & pair);

private slots:
    void SlotPowerOff(void);
    void SlotPowerOn(void);
    void SlotHome(void);
    void SlotArmCurrentStateEventHandler(PairStringType armState);
    void SlotTeleopEnable(bool toggle);
    void SlotTeleopToggle(void);
    void SlotTeleopStart(void);
    void SlotTeleopStop(void);
    void SlotTeleopEnabledEventHandler(bool enabled);
    void SlotTeleopPSMSelectedEventHandler(PairStringType selected);
    void SlotTeleopPSMUnselectedEventHandler(PairStringType unselected);
    void SlotSetScale(double scale);
    void SlotScaleEventHandler(double scale);
    void SlotOperatorPresentEventHandler(bool operatorPresent);
    void SlotClutchEventHandler(bool clutch);
    void SlotCameraEventHandler(bool camera);
    void SlotSetVolume(void);
    void SlotVolumeEventHandler(double volume);
    void SlotEnableDirectControl(bool toggle);
    void SlotEmulateOperatorPresent(bool toggle);
    void SlotEmulateClutch(bool toggle);
    void SlotEmulateCamera(bool toggle);
    void SlotComponentViewer(void);

protected:
    void closeEvent(QCloseEvent * event);

    void setupUi(void);

    struct MainStruct {
        mtsFunctionVoid power_off;
        mtsFunctionVoid power_on;
        mtsFunctionVoid home;
        mtsFunctionWrite teleop_enable;
        mtsFunctionWrite select_teleop_PSM;
        mtsFunctionWrite set_scale;
        mtsFunctionWrite set_volume;
        mtsFunctionWrite emulate_operator_present;
        mtsFunctionWrite emulate_clutch;
        mtsFunctionWrite emulate_camera;
        mtsFunctionRead calibration_mode;
    } Console;

    void ArmCurrentStateEventHandler(const prmKeyValue & armState);
    void TeleopEnabledEventHandler(const bool & enabled);
    void TeleopPSMSelectedEventHandler(const prmKeyValue & selected);
    void TeleopPSMUnselectedEventHandler(const prmKeyValue & unselected);
    void ScaleEventHandler(const double & scale);
    void OperatorPresentEventHandler(const prmEventButton & button);
    void ClutchEventHandler(const prmEventButton & button);
    void CameraEventHandler(const prmEventButton & button);
    void VolumeEventHandler(const double & volume);

    QVBoxLayout * QVBArms;
    std::map<QString, QPushButton *> ArmButtons;

    QVBoxLayout * QVBTeleops;
    std::map<QString, std::pair<QPushButton *, QCheckBox *> > TeleopButtons;

    QPushButton * QPBPowerOff;
    QPushButton * QPBPowerOn;
    QPushButton * QPBHome;
    QPushButton * QPBTeleopEnable;
    QCheckBox * QCBTeleopEnable;
    QDoubleSpinBox * QSBScale;
    QRadioButton * QRBOperatorPresent;
    QRadioButton * QRBClutch;
    QRadioButton * QRBCamera;
    QSlider * QSVolume;

    QCheckBox * QCBEnableDirectControl;
    QPushButton * QPBComponentViewer;
    QTabWidget * QTWidgets;
    mtsMessageQtWidget * QMMessage;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitConsoleQtWidget);

#endif // _mtsIntuitiveResearchKitConsoleQtWidget_h
