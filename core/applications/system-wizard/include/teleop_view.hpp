/*
  Author(s):  Brendan Burkhart
  Created on: 2025-06-06

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_TELEOP_VIEW
#define SYSTEM_WIZARD_TELEOP_VIEW

#include "list_view.hpp"
#include "models/config_model.hpp"

namespace system_wizard {

class TeleopView : public ItemView {
public:
    TeleopView(SystemConfigModel& model, ListView& list_view, int id, QWidget* parent = nullptr);

    void updateData(int id) override;

private:
    SystemConfigModel* model;
    QLabel* display;
};

class TeleopViewFactory : public ItemViewFactory {
public:
    TeleopViewFactory(SystemConfigModel& model);

    TeleopView* create(int id, ListView& list_view);

private:
    SystemConfigModel* model;
};

}

#endif // SYSTEM_WIZARD_TELEOP_VIEW
