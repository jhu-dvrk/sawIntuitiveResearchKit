/*
  Author(s):  Brendan Burkhart
  Created on: 2025-07-28

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_SYSTEM_LAUNCHER
#define SYSTEM_WIZARD_SYSTEM_LAUNCHER

#include <QApplication>
#include <QtCore>
#include <optional>

#include "models/config_model.hpp"

namespace system_wizard {

class SystemLauncher : public QObject {
    Q_OBJECT

public:
    SystemLauncher(QCoreApplication& app);

    std::string launch(const SystemConfigModel& model);

    bool isRunning();

signals:
    void error(std::string message);
    void stateChanged();

private:
    std::optional<QString> findSystem();
    int getROSVersion();

    void onFinished(int exit_code, QProcess::ExitStatus status);
    void onStateChanged(QProcess::ProcessState new_state);
    void onError(QProcess::ProcessError error);

    QCoreApplication* application;

    QProcess process;
    QTemporaryFile temp_config_file;
};

}

#endif // SYSTEM_WIZARD_SYSTEM_LAUNCHER
