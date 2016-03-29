/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitConsoleQtWidget_h
#define _mtsIntuitiveResearchKitConsoleQtWidget_h

#include <cisstMultiTask/mtsComponent.h>

class QPushButton;
class QTextEdit;
class QTabWidget;

#include <QWidget>

class mtsIntuitiveResearchKitConsoleQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

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
    void SignalAppendMessage(QString);
    void SignalSetColor(QColor);

private slots:
    void SlotPowerOff(void);
    void SlotHome(void);
    void SlotTeleopStart(void);
    void SlotTeleopStop(void);
    void SlotTextChanged(void);

protected:
    void closeEvent(QCloseEvent * event);

    void setupUi(void);

    struct MainStruct {
        mtsFunctionVoid PowerOff;
        mtsFunctionVoid Home;
        mtsFunctionWrite TeleopEnable;
    } Console;

    void ErrorEventHandler(const std::string & message);
    void WarningEventHandler(const std::string & message);
    void StatusEventHandler(const std::string & message);

    QPushButton * QPBPowerOff;
    QPushButton * QPBHome;
    QPushButton * QPBTeleopStart;
    QPushButton * QPBTeleopStop;
    QTabWidget * QTWidgets;
    QTextEdit * QTEMessages;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitConsoleQtWidget);

#endif // _mtsIntuitiveResearchKitConsoleQtWidget_h
