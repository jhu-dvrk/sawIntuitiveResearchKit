/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-17

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_MAINWINDOW_HPP
#define SYSTEM_WIZARD_MAINWINDOW_HPP

#include <QMainWindow>
#include <QMenu>

#include "config_sources.hpp"

namespace system_wizard {

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow();

private slots:
    void open_folder();

private:
    void createActions();
    void createMenus();

    ConfigSources* config_sources;

    QMenu* file_menu;
    QAction* open_act;
};

}

#endif // SYSTEM_WIZARD_MAINWINDOW_HPP
