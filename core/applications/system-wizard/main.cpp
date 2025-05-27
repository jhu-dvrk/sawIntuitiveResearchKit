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

#include <QApplication>

#include "include/mainwindow.hpp"

int main(int argc, char** argv)
{
    QApplication application(argc, argv);
    system_wizard::MainWindow window;
    window.showMaximized();

    return application.exec();
}
