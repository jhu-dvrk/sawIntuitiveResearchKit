/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-27

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_ARM_VIEW
#define SYSTEM_WIZARD_ARM_VIEW

#include "config_model.hpp"
#include "list_view.hpp"

namespace system_wizard {

class ArmView : public ItemView {
public:
    ArmView(SystemConfigModel& model, ListView& list_view, int id, QWidget* parent = nullptr);

    void updateData(int id) override;

private:
    SystemConfigModel* model;
    QLabel* display;
};

class ArmViewFactory : public ItemViewFactory {
public:
    ArmViewFactory(SystemConfigModel& model);

    ArmView* create(int id, ListView& list_view);

private:
    SystemConfigModel* model;
};

}

#endif // SYSTEM_WIZARD_ARM_VIEW
