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
        inline QTabWidget * get_tab_widget(void) {
            return QTWidgets;
        }
        
    signals:
        void SignalArmCurrentState(PairStringType armState);
        void SignalVolume(double volume);
        
    protected:
        void FocusArmButton(const QString & armName);
                                                             
    private slots:
        void SlotPowerOff(void);
        void SlotPowerOn(void);
        void SlotHome(void);
        void SlotArmCurrentStateEventHandler(PairStringType armState);
        void SlotSetVolume(void);
        void SlotVolumeEventHandler(double volume);
        void SlotComponentViewer(void);
        
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
        
        void arm_current_state_event_handler(const prmKeyValue & armState);
        void volume_event_handler(const double & volume);
        
        QVBoxLayout * QVBArms;
        std::map<QString, QPushButton *> ArmButtons;
        
        QPushButton * QPBPowerOff;
        QPushButton * QPBPowerOn;
        QPushButton * QPBHome;
        QSlider * QSVolume;
        
        QPushButton * QPBComponentViewer;
        QTabWidget * QTWidgets;
        mtsMessageQtWidget * QMMessage;
    };
}

CMN_DECLARE_SERVICES_INSTANTIATION(dvrk::system_Qt_widget);

#endif // _dvrk_system_Qt_widget_h
