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
#include <filesystem>

#include "include/mainwindow.hpp"
#include "system_launcher.hpp"

int main(int argc, char** argv)
{
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication application(argc, argv);

    cmnCommandLineOptions options;
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");
    std::string config_file;
    options.AddOptionOneValue("c", "config",
                              "config file to open in editor",
                              cmnCommandLineOptions::OPTIONAL_OPTION,
                              &config_file);

    if (!options.Parse(argc, argv, std::cerr)) {
        return 1;
    }

    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    if (options.IsSet("config")) {
        if (!std::filesystem::is_regular_file(config_file)) {
            std::cerr << "Cannot find config file \"" << config_file << "\"" << std::endl;
            return 1;
        }
    }

    QLocale::setDefault(QLocale::English);
    application.setWindowIcon(QIcon(":/dVRK.png"));
    cmnQt::QApplicationExitsOnCtrlC();

    config_wizard::SystemLauncher launcher(application, options.IsSet("dark-mode"));

    config_wizard::MainWindow window(launcher);
    if (options.IsSet("config")) {
        window.openConfigFile(config_file);
    } else {
        window.newConfig();
    }

    window.showMaximized();

    return application.exec();
}
