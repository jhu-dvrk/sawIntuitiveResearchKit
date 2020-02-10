/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

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

#include <QWidget>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

class CISST_EXPORT mtsIntuitiveResearchKitConsoleQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

    typedef QPair<QString, QString> ArmCurrentStateType;
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
    void SignalArmCurrentState(ArmCurrentStateType armState);
    void SignalScale(double scale);
    void SignalClutch(bool clutch);
    void SignalOperatorPresent(bool operatorPresent);
    void SignalCamera(bool camera);

private slots:
    void SlotPowerOff(void);
    void SlotPowerOn(void);
    void SlotHome(void);
    void SlotArmCurrentStateEventHandler(ArmCurrentStateType armState);
    void SlotTeleopStart(void);
    void SlotTeleopStop(void);
    void SlotSetScale(double scale);
    void SlotScaleEventHandler(double scale);
    void SlotClutchEventHandler(bool clutch);
    void SlotOperatorPresentEventHandler(bool operatorPresent);
    void SlotCameraEventHandler(bool camera);
    void SlotSetVolume(void);
    void SlotComponentViewer(void);
    void SlotArmButton(const QString & armName);

protected:
    void closeEvent(QCloseEvent * event);

    void setupUi(void);

    struct MainStruct {
        mtsFunctionVoid PowerOff;
        mtsFunctionVoid PowerOn;
        mtsFunctionVoid Home;
        mtsFunctionWrite TeleopEnable;
        mtsFunctionWrite SetScale;
        mtsFunctionWrite SetVolume;
    } Console;

    void ArmCurrentStateEventHandler(const prmKeyValue & armState);
    void ScaleEventHandler(const double & scale);
    void ClutchEventHandler(const prmEventButton & button);
    void OperatorPresentEventHandler(const prmEventButton & button);
    void CameraEventHandler(const prmEventButton & button);

    QVBoxLayout * QVBArms;
    std::map<std::string, QPushButton *> ArmButtons;

    QPushButton * QPBPowerOff;
    QPushButton * QPBPowerOn;
    QPushButton * QPBHome;
    QPushButton * QPBTeleopStart;
    QPushButton * QPBTeleopStop;
    QDoubleSpinBox * QSBScale;
    QRadioButton * QRBClutch;
    QRadioButton * QRBOperatorPresent;
    QRadioButton * QRBCamera;
    QSlider * QSVolume;

    QPushButton * QPBComponentViewer;
    QTabWidget * QTWidgets;
    mtsMessageQtWidget * QMMessage;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitConsoleQtWidget);

#endif // _mtsIntuitiveResearchKitConsoleQtWidget_h
