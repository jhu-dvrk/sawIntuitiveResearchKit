/*
  Author(s):  Brendan Burkhart
  Created on: 2025-07-27

  (C) Copyright 2025-2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "file_selector.hpp"

#include <filesystem>

namespace config_wizard {

FileSelector::FileSelector(QWidget* parent) : QWidget(parent){
    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setMargin(0);
    layout->setAlignment(Qt::AlignCenter);

    dialog = new QFileDialog(this);
    dialog->setFileMode(QFileDialog::ExistingFile);
    dialog->setViewMode(QFileDialog::List);
    dialog->setOptions(QFileDialog::DontResolveSymlinks);
    dialog->setNameFilter("Config file (*.json)");

    display = new QLineEdit();
    display->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    browse_button = new QPushButton("Browse");
    QObject::connect(browse_button, &QPushButton::clicked, this, [this]() {
        if (current_file.has_value()) {
            std::filesystem::path p = current_file.value();
            dialog->setDirectory(QString::fromStdString(p.parent_path().generic_string()));
        } else if (starting_directory.has_value()) {
            dialog->setDirectory(QString::fromStdString(starting_directory->string()));
        }

        dialog->open();
    });
    QObject::connect(dialog, &QFileDialog::fileSelected, this, [this](const QString& file_name) {
        if (file_name.isEmpty()) { return; }
        setCurrentFile(file_name.toStdString());
        emit selected(file_name.toStdString());
    });

    layout->addWidget(display);
    layout->addWidget(browse_button);
}

void FileSelector::setStartingDirectory(std::filesystem::path dir) {
    starting_directory = dir;
}

void FileSelector::setReferenceDirectory(std::filesystem::path dir) {
    reference_directory = dir;
    if (!starting_directory.has_value()) {
        starting_directory = dir;
    }
}

void FileSelector::setCurrentFile(std::string file) {
    std::filesystem::path file_path = file;
    if (reference_directory.has_value() && file_path.is_relative()) {
        file_path = reference_directory.value() / file_path;
    }

    current_file = file_path.generic_string();

    std::string name = currentRelativeFile().value();
    display->setText(QString::fromStdString(name));
}

std::optional<std::string> FileSelector::currentFile() const {
    return current_file;
}

std::optional<std::string> FileSelector::currentRelativeFile() const {
    if (!current_file) {
        return {};
    }

    if (reference_directory.has_value()) {
        bool inside_dir = isChild(current_file.value(), reference_directory.value());
        if (inside_dir) {
            return std::filesystem::proximate(current_file.value(), reference_directory.value()).generic_string();
        }
    }

    return current_file;
}

bool FileSelector::isChild(std::filesystem::path p, std::filesystem::path base) {
    std::filesystem::path rel = std::filesystem::relative(p, base);
    return !rel.empty() && *rel.begin() != "..";
}

}
