/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-28

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef CONFIG_WIZARD_IO_VIEW
#define CONFIG_WIZARD_IO_VIEW

#include "list_view.hpp"
#include "models/config_model.hpp"

namespace config_wizard {

class IOView : public ItemView {
public:
    IOView(SystemConfigModel& model, ListView& list_view, int id, QWidget* parent = nullptr);

    void updateData(int id) override;

private:
    SystemConfigModel* model;
    QLabel* display;
};

}

#endif // CONFIG_WIZARD_IO_VIEW
