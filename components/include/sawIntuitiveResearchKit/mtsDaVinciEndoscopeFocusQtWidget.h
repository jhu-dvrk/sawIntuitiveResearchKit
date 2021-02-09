/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2020-0210

  (C) Copyright 2020-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsDaVinciEndoscopeFocusQtWidget_h
#define _mtsDaVinciEndoscopeFocusQtWidget_h

#include <cisstMultiTask/mtsComponent.h>
#include <QWidget>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>


class QPushButton;
class QLabel;

class CISST_EXPORT mtsDaVinciEndoscopeFocusQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsDaVinciEndoscopeFocusQtWidget(const std::string & componentName);
    ~mtsDaVinciEndoscopeFocusQtWidget(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

signals:
    void SignalLocked(bool locked);
    void SignalFocusingIn(bool focusing);
    void SignalFocusingOut(bool focusing);

private slots:
    // to set from the GUI
    void SlotLock(void);
    void SlotFocusIn(bool focus);
    void SlotFocusOut(bool focus);

    // to update GUI from component's events
    void SlotLockedEventHandler(bool locked);
    void SlotFocusingInEventHandler(bool focusing);
    void SlotFocusingOutEventHandler(bool focusing);

private:
    //! setup DaVinciEndoscopeFocus controller GUI
    void setupUi(void);
    void UpdateLock(const bool & locked);

    void LockedEventHandler(const bool & locked);
    void FocusingInEventHandler(const bool & focusing);
    void FocusingOutEventHandler(const bool & focusing);

protected:
    struct {
        mtsFunctionWrite lock;
        mtsFunctionWrite focus_in;
        mtsFunctionWrite focus_out;
    } Endoscope;

private:
    QLabel * QLFocusingIn;
    QLabel * QLFocusingOut;

    QPushButton * QPBLock;
    QPushButton * QPBFocusIn;
    QPushButton * QPBFocusOut;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDaVinciEndoscopeFocusQtWidget);

#endif // _mtsDaVinciEndoscopeFocusQtWidget_h
