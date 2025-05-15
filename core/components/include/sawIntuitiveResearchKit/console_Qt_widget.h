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


#ifndef _dvrk_console_Qt_widget_h
#define _dvrk_console_Qt_widget_h

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

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

namespace dvrk {

    class CISST_EXPORT console_Qt_widget: public QWidget, public mtsComponent
    {
        Q_OBJECT;
        CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

    public:
        console_Qt_widget(const std::string & componentName);
        inline virtual ~console_Qt_widget() {}

        void Configure(const std::string & filename = "");
        void Startup(void);
        void Cleanup(void);
        void HasTeleOp(const bool & hasTeleOp);

    signals:
        void signal_teleop_selected(QString selected);
        void signal_teleop_unselected(QString unselected);
        void SignalTeleopEnabled(bool toggle);
        void SignalScale(double scale);
        void SignalOperatorPresent(bool operatorPresent);
        void signal_clutch(bool clutch);
        void SignalCamera(bool camera);

    protected:
        void get_teleop_button_check(const QString & pair, QPushButton * & button, QCheckBox * & check);
        void select_teleop_check(const QString & pair);
        void unselect_teleop_check(const QString & pair);

    private slots:
        void SlotTeleopEnable(bool toggle);
        void SlotTeleopToggle(void);
        void SlotTeleopStart(void);
        void SlotTeleopStop(void);
        void SlotTeleopEnabledEventHandler(bool enabled);
        void slot_teleop_selected_event_handler(QString selected);
        void slot_teleop_unselected_event_handler(QString unselected);
        void SlotSetScale(double scale);
        void SlotScaleEventHandler(double scale);
        void SlotOperatorPresentEventHandler(bool operatorPresent);
        void slot_clutched(bool clutch);
        void SlotCameraEventHandler(bool camera);
        void SlotEnableDirectControl(bool toggle);
        void SlotEmulateOperatorPresent(bool toggle);
        void SlotEmulateClutch(bool toggle);
        void SlotEmulateCamera(bool toggle);

    protected:

        void setupUi(void);

        struct {
            mtsFunctionWrite teleop_enable;
            mtsFunctionWrite select_teleop;
            mtsFunctionWrite unselect_teleop;
            mtsFunctionWrite set_scale;
            mtsFunctionWrite emulate_operator_present;
            mtsFunctionWrite emulate_clutch;
            mtsFunctionWrite emulate_camera;
        } console;

        void TeleopEnabledEventHandler(const bool & enabled);
        void teleop_selected_event_handler(const std::string & selected);
        void teleop_unselected_event_handler(const std::string & unselected);
        void ScaleEventHandler(const double & scale);
        void OperatorPresentEventHandler(const prmEventButton & button);
        void clutch_event_handler(const prmEventButton & button);
        void camera_event_handler(const prmEventButton & button);

        QVBoxLayout * QVBTeleops;
        std::map<QString, std::pair<QPushButton *, QCheckBox *>> m_teleop_buttons;

        QPushButton * QPBTeleopEnable;
        QCheckBox * QCBTeleopEnable;
        QDoubleSpinBox * QSBScale;
        QRadioButton * QRBOperatorPresent;
        QRadioButton * QRBClutch;
        QRadioButton * QRBCamera;

        QCheckBox * QCBEnableDirectControl;
        QPushButton * QPBComponentViewer;
        QTabWidget * QTWidgets;
    };
}

CMN_DECLARE_SERVICES_INSTANTIATION(dvrk::console_Qt_widget);

#endif // _dvrk_console_Qt_widget_h
