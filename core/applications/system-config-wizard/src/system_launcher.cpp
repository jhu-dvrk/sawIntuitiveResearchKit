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

#include "system_launcher.hpp"

#include <QtCore>

#include "models/config_model.hpp"

namespace config_wizard {

SystemLauncher::SystemLauncher(QCoreApplication& app, bool dark_mode) : QObject(), application(&app), dark_mode(dark_mode), process(this) {
    QObject::connect(&process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this, &SystemLauncher::onFinished);
    QObject::connect(&process, &QProcess::errorOccurred, this, &SystemLauncher::onError);
    QObject::connect(&process, &QProcess::stateChanged, this, &SystemLauncher::onStateChanged);
}

std::string SystemLauncher::launch(const SystemConfigModel& model) {
    if (process.state() == QProcess::Running) {
        return "System already running";
    }

    if (!temp_config_file.open()) {
        return "Failed to create temporary config file";
    }

    bool ok = model.save(temp_config_file.fileName().toStdString());
    if (!ok) {
        return "Failed to save temporary config file";
    }

    QString program;
    QStringList arguments;

    // run either rosrun, ros2 run, or plain sawIntuitiveResearchKit based on what user's environment is
    int ROS = getROSVersion();
    if (ROS == 1) { // unsupported?
        program = QStandardPaths::findExecutable("rosrun");
        if (program.isEmpty()) {
            return "Executable \"rosrun\" not found in path";
        }
        arguments << "dvrk_robot" << "dvrk_system";
    } else if (ROS == 2) {
        program = QStandardPaths::findExecutable("ros2");
        if (program.isEmpty()) {
            return "Executable \"ros2\" not found in path";
        }
        arguments << "run" << "dvrk_robot" << "dvrk_system";
    } else {
        std::optional<QString> system = findSystem();
        if (!system) {
            return "Executable \"sawIntuitiveResearchKitSystem\" not found";
        }
        program = system.value();
    }

    arguments << "-j" << temp_config_file.fileName();

    if (dark_mode) {
        arguments << "-D";
    }

    process.start(program, arguments);
    return "";
}

std::optional<QString> SystemLauncher::findSystem() {
    // search install/binary directory
    QStringList bin_paths = QStringList(application->applicationDirPath());
    QString system_executable = QStandardPaths::findExecutable("sawIntuitiveResearchKitSystem", bin_paths);
    if (!system_executable.isEmpty()) {
        return system_executable;
    }

    // fallback and search system paths
    system_executable = QStandardPaths::findExecutable("sawIntuitiveResearchKitSystem");
    if (!system_executable.isEmpty()) {
        return system_executable;
    }

    return {};
}

bool SystemLauncher::isRunning() {
    return process.state() == QProcess::Running;
}

int SystemLauncher::getROSVersion() {
    /** Returns 1 or 2 if ROS 1/ROS 2 is sourced, otherwise -1 */
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    QString ros_version = env.value("ROS_VERSION", "");
    if (ros_version == "1") {
        return 1;
    } else if (ros_version == "2") {
        return 2;
    } else {
        return -1;
    }
}

void SystemLauncher::onFinished(int exit_code, QProcess::ExitStatus status) {
    if (status == QProcess::NormalExit && exit_code == 0) {
        return;
    }

    std::string log_info = "see dvrk_system_stdout.log/dvrk_system_stderr.log for details";
    if (status == QProcess::NormalExit) {
        emit error("System has exited with code " + std::to_string(exit_code) + ", " + log_info);
    } else {
        emit error("System has crashed, " + log_info);
    }

    // dump stdout/stderr to disk so user can review later if desired
    QFile stdout("dvrk_system_stdout.log");
    if (stdout.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        QTextStream stream(&stdout);
        stream << process.readAllStandardOutput();
    }

    QFile stderr("dvrk_system_stderr.log");
    if (stderr.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        QTextStream stream(&stderr);
        stream << process.readAllStandardError();
    }
}

void SystemLauncher::onStateChanged(QProcess::ProcessState) {
    emit stateChanged();
}

void SystemLauncher::onError(QProcess::ProcessError err) {
    switch (err) {
    case QProcess::FailedToStart:
        emit error("dVRK failed to start");
        break;
    case QProcess::Crashed:
        emit error("dVRK crashed");
        break;
    case QProcess::Timedout:
        emit error("dVRK timed out");
        break;
    case QProcess::WriteError:
        emit error("dVRK write error");
        break;
    case QProcess::ReadError:
        emit error("dVRK read error");
        break;
    default:
        emit error("dVRK failed with unknown error");
    }
}

}
