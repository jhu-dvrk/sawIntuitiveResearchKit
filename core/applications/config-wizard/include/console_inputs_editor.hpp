/*
  Author(s):  Brendan Burkhart
  Created on: 2025-07-06

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef CONFIG_WIZARD_CONSOLE_INPUTS_EDITOR
#define CONFIG_WIZARD_CONSOLE_INPUTS_EDITOR

#include <QtWidgets>

#include "models/config_model.hpp"
#include "models/enum_list_model.hpp"

namespace config_wizard {

class ConsoleInputsEditor : public QWidget {
    Q_OBJECT

public:
    ConsoleInputsEditor(ConsoleInputConfig& model, ListModelT<ArmConfig>& arms, QWidget* parent = nullptr);

private:
    ConsoleInputConfig* model;
    ListModelT<ArmConfig>* arms;

    EnumListModel<ConsoleInputType> input_type_model;
    EnumListModel<HeadSensorType> head_sensor_type_model;

    QComboBox* pedals_available_mtms;
    QComboBox* head_sensor_available_mtms;

    QComboBox* available_forcedimensions;

    void updateAvailableArms();
};

}

#endif // CONFIG_WIZARD_CONSOLE_INPUTS_EDITOR
