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

#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>

#include "include/mainwindow.hpp"

int main(int argc, char** argv)
{
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication application(argc, argv);

    cmnCommandLineOptions options;
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");
    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }

    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    QLocale::setDefault(QLocale::English);
    application.setWindowIcon(QIcon(":/dVRK.png"));
    cmnQt::QApplicationExitsOnCtrlC();

    system_wizard::MainWindow window;
    window.showMaximized();

    return application.exec();
}
