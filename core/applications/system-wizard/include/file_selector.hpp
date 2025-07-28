/*
  Author(s):  Brendan Burkhart
  Created on: 2025-07-27

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_FILE_SELECTOR
#define SYSTEM_WIZARD_FILE_SELECTOR

#include <QtWidgets>

#include <filesystem>
#include <optional>

namespace system_wizard {

class FileSelector : public QWidget {
    Q_OBJECT

public:
    FileSelector(QWidget* parent = nullptr);

    void setStartingDirectory(std::filesystem::path directory);
    void setCurrentFile(std::string file);
    std::optional<std::string> currentFile() const;

signals:
    void selected(std::filesystem::path);

private:
    QFileDialog* dialog;
    QPushButton* browse_button;
    QLineEdit* display;

    std::optional<std::string> current_file;
    std::filesystem::path starting_directory;
};

}

#endif // SYSTEM_WIZARD_FILE_SELECTOR
