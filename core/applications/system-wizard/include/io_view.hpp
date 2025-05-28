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

#ifndef SYSTEM_WIZARD_IO_VIEW
#define SYSTEM_WIZARD_IO_VIEW

#include "config_model.hpp"
#include "list_view.hpp"

namespace system_wizard {

class IOView : public ItemView {
    Q_OBJECT

public:
    IOView(SystemConfigModel* model, ListView& list_view, int id, QWidget* parent = nullptr);

public slots:
    void updateData(int id) override;

private:
    SystemConfigModel* model;
    QLabel* display;
};

class IOViewFactory : public ItemViewFactory {
public:
    IOViewFactory(SystemConfigModel* model);

    IOView* create(int id, ListView& list_view);

private:
    SystemConfigModel* model;
};

}

#endif // SYSTEM_WIZARD_IO_VIEW
