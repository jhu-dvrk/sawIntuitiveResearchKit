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


#ifndef _dvrk_system_Qt_widget_h
#define _dvrk_system_Qt_widget_h

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmKeyValue.h>

class QPushButton;
class QTabWidget;
class QSlider;
class QVBoxLayout;

#include <QWidget>

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

namespace dvrk {

    class console_Qt_widget;

    class CISST_EXPORT system_Qt_widget: public QWidget, public mtsComponent
    {
        Q_OBJECT;
        CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

        typedef QPair<QString, QString> PairStringType;
    public:
        system_Qt_widget(const std::string & componentName);
        inline virtual ~system_Qt_widget() {}

        void Configure(const std::string & filename = "");
        void Startup(void);
        void Cleanup(void);

        inline QTabWidget * get_components_tab(void) {
            return QTComponents;
        }

        inline QTabWidget * get_consoles_tab(void) {
            return QTConsoles;
        }

    signals:
        void signal_arm_current_state(PairStringType _arm_state);
        void signal_volume(double volume);

    protected:
        void focus_arm_button(const QString & _arm_name);

    private slots:
        void slot_power_off(void);
        void slot_power_on(void);
        void slot_home(void);
        void slot_arm_current_state_event_handler(PairStringType _arm_state);
        void slot_set_volume(void);
        void slot_volume_event_handler(double _volume);
        void slot_component_viewer(void);

    protected:
        void closeEvent(QCloseEvent * event);

        void setupUi(void);

        struct {
            mtsFunctionVoid power_off;
            mtsFunctionVoid power_on;
            mtsFunctionVoid home;
            mtsFunctionWrite set_volume;
            mtsFunctionRead calibration_mode;
        } system;

        void arm_current_state_event_handler(const prmKeyValue & _arm_state);
        void volume_event_handler(const double & volume);

        QVBoxLayout * QVBArms;
        std::map<QString, QPushButton *> m_arm_buttons;

        QPushButton * QPBPowerOff;
        QPushButton * QPBPowerOn;
        QPushButton * QPBHome;
        QSlider * QSVolume;

        QVBoxLayout * QVBConsole;
        std::map<QString, dvrk::console_Qt_widget *> m_console_widgets;

        QPushButton * QPBComponentViewer;
        QTabWidget * QTComponents;
        QTabWidget * QTConsoles;
        mtsMessageQtWidget * QMMessage;
    };
}

CMN_DECLARE_SERVICES_INSTANTIATION(dvrk::system_Qt_widget);

#endif // _dvrk_system_Qt_widget_h
