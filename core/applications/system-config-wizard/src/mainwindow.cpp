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

#include "mainwindow.hpp"

#include <QtWidgets>
#include <QFileDialog>
#include <QLabel>

namespace config_wizard {

MainWindow::MainWindow(SystemLauncher& launcher) : directory_chooser(this, "Open config source folder") {
    directory_chooser.setFileMode(QFileDialog::Directory);
    directory_chooser.setViewMode(QFileDialog::List);
    directory_chooser.setOptions(QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    config_sources = new ConfigSources();

    editor = new Editor(*config_sources, launcher);
    QObject::connect(&launcher, &SystemLauncher::error, this, [this](std::string message) {
        if (!message.empty()) {
            QMessageBox::critical(this, "dVRK System Error", QString::fromStdString(message));
        }
    } );

    QSplitter *splitter = new QSplitter();
    splitter->addWidget(config_sources);
    splitter->setCollapsible(0, false);

    splitter->addWidget(editor);
    splitter->setCollapsible(1, false);

    setCentralWidget(splitter);

    createActions();
    createMenus();

    setWindowTitle("dVRK Config Wizard");
}

void MainWindow::newConfig() {
    editor->newConfig();
}

void MainWindow::openConfigFile(std::filesystem::path config) {
    std::filesystem::path source_dir = config.parent_path();
    config_sources->addSource(QDir(QString::fromStdString(source_dir.string())));
    editor->openConfigFile(config);
}

void MainWindow::createActions() {
    open_source = new QAction(style()->standardIcon(QStyle::SP_DirOpenIcon),
                         "Open source folder", this);
    open_source->setShortcuts({QKeySequence(Qt::CTRL | Qt::Key_K, Qt::CTRL | Qt::Key_O)});
    open_source->setStatusTip("Open config source directory");
    connect(open_source, &QAction::triggered, this, &MainWindow::open_folder);
    connect(&directory_chooser, &QDialog::accepted, this, &MainWindow::folder_chosen);

    new_config = new QAction(style()->standardIcon(QStyle::SP_FileIcon),
                         "New config file", this);
    new_config->setShortcuts(QKeySequence::New);
    new_config->setStatusTip("New config file");
    connect(new_config, &QAction::triggered, editor, &Editor::newConfig);

    open_config = new QAction(style()->standardIcon(QStyle::SP_FileIcon),
                         "Open config file", this);
    open_config->setShortcuts(QKeySequence::Open);
    open_config->setStatusTip("Open config file");
    connect(open_config, &QAction::triggered, editor, &Editor::openConfig);

    save_config = new QAction(style()->standardIcon(QStyle::SP_DriveFDIcon),
                         "Save config file", this);
    save_config->setShortcuts(QKeySequence::Save);
    save_config->setStatusTip("Save config file");
    connect(save_config, &QAction::triggered, editor, &Editor::save);

    save_config_as = new QAction(style()->standardIcon(QStyle::SP_DriveFDIcon),
                         "Save as", this);
    save_config_as->setShortcuts(QKeySequence::SaveAs);
    save_config_as->setStatusTip("Save as");
    connect(save_config_as, &QAction::triggered, editor, &Editor::saveAs);
}

void MainWindow::createMenus() {
    file_menu = menuBar()->addMenu("&File");
    file_menu->addAction(open_source);
    file_menu->addAction(new_config);
    file_menu->addAction(open_config);
    file_menu->addAction(save_config);
    file_menu->addAction(save_config_as);
}

void MainWindow::open_folder() {
    directory_chooser.open();

    auto source_dir = config_sources->dir();
    if (source_dir.has_value()) {
        directory_chooser.setDirectory(QString::fromStdString(source_dir.value()));
    } else {
        directory_chooser.setDirectory(QDir::current());
    }
}

void MainWindow::folder_chosen() {
    QStringList file_names = directory_chooser.selectedFiles();
    if (file_names.count() != 1) {
        return;
    }

    QString selected_directory = file_names.at(0);
    if (selected_directory.isEmpty()) {
        return;
    }

    config_sources->addSource(selected_directory);
}

}
