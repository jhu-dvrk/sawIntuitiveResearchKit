/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2021-11-29

  (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsDerivedPSMQtWidget_h
#define _mtsDerivedPSMQtWidget_h

#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsComponent.h>
#include <QWidget>

class QCheckBox;
class QDoubleSpinBox;

class mtsDerivedPSMQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsDerivedPSMQtWidget(const std::string & componentName);
    mtsDerivedPSMQtWidget(const mtsComponentConstructorArg & arg);
    ~mtsDerivedPSMQtWidget(){}

    void Configure(const std::string & filename = "") override;
    void Startup(void) override;
    void Cleanup(void) override;


signals:
    void SignalGain(double gain);
    void SignalActivated(bool lock);

private slots:
    void timerEvent(QTimerEvent * event);
    // to set from the GUI
    void SlotSetGain(double gain);
    void SlotActivate(bool activate);

    // to update GUI from component's events
    void SlotGainEventHandler(double gain);
    void SlotActivatedEventHandler(bool activated);

private:
    //! setup GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

    void GainEventHandler(const double & gain);
    void ActivatedEventHandler(const bool & activated);

protected:
    void Init(void);
    mtsFunctionWrite set_gain;
    mtsFunctionWrite activate;

private:
    QCheckBox * QCBActivate;
    QDoubleSpinBox * QSBGain;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDerivedPSMQtWidget);

#endif // _mtsDerivedPSMQtWidget_h
