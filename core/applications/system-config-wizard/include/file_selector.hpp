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

#ifndef CONFIG_WIZARD_FILE_SELECTOR
#define CONFIG_WIZARD_FILE_SELECTOR

#include <QtWidgets>

#include <filesystem>
#include <optional>

namespace config_wizard {

class FileSelector : public QWidget {
    Q_OBJECT

public:
    FileSelector(QWidget* parent = nullptr);

    /** Where the file picker will open if not file is selected, if set */
    void setStartingDirectory(std::filesystem::path directory);

    /** Files will be displayed/referenced relative to this folder if set */
    void setReferenceDirectory(std::filesystem::path directory);

    /** Can be either absolute path, or relative to reference dir */
    void setCurrentFile(std::string file);

    /** Absolute path to current file, if any */
    std::optional<std::string> currentFile() const;

    /** If child of reference dir, then relative to reference, otherwise full path */
    std::optional<std::string> currentRelativeFile() const;

signals:
    void selected(std::filesystem::path);

private:
    static bool isChild(std::filesystem::path p, std::filesystem::path base);

    QFileDialog* dialog;
    QPushButton* browse_button;
    QLineEdit* display;

    std::optional<std::string> current_file;
    std::optional<std::filesystem::path> starting_directory;
    std::optional<std::filesystem::path> reference_directory;
};

}

#endif // CONFIG_WIZARD_FILE_SELECTOR
