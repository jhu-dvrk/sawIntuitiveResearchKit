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

#include "file_selector.hpp"

namespace system_wizard {

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
    QObject::connect(browse_button, &QPushButton::clicked, dialog, &QDialog::open);
    QObject::connect(dialog, &QFileDialog::fileSelected, this, [this](const QString& file_name) {
        if (file_name.isEmpty()) { return; }
        display->setText(file_name);
        emit selected(file_name.toStdString());
    });

    layout->addWidget(display);
    layout->addWidget(browse_button);
}

void FileSelector::setStartingDirectory(std::filesystem::path file) {
    dialog->setDirectory(QString::fromStdString(file.string()));
}

void FileSelector::setCurrentFile(std::string file) {
    current_file = file;
    display->setText(QString::fromStdString(file));
    dialog->setDirectory(QString::fromStdString(file));
}

std::optional<std::string> FileSelector::currentFile() const {
    return current_file;
}

}
