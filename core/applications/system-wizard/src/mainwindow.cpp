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

#include "config_editor.hpp"
#include "config_model.hpp"

#include <QtWidgets>
#include <QFileDialog>
#include <QLabel>

namespace system_wizard {

MainWindow::MainWindow() {
    SystemConfigModel* model1 = new SystemConfigModel();

    ConfigEditor* editor1 = new ConfigEditor(model1);

    SystemConfigModel* model2 = new SystemConfigModel();
    ConfigEditor* editor2 = new ConfigEditor(model2);

    QTabWidget* editor = new QTabWidget();
    editor->addTab(editor1, "Editor 1");
    editor->addTab(editor2, "Editor 2");

    config_sources = new ConfigSources();

    QSplitter *splitter = new QSplitter();
    splitter->addWidget(config_sources);
    splitter->setCollapsible(0, false);

    splitter->addWidget(editor);
    splitter->setCollapsible(1, false);

    setCentralWidget(splitter);

    createActions();
    createMenus();

    setWindowTitle("dVRK System Wizard");

    model1->arm_configs.addItem(ArmConfig("PSM1", ArmType(ArmType::Value::PSM_GENERIC)));
    model1->arm_configs.addItem(ArmConfig("ECM", ArmType(ArmType::Value::ECM)));

    model2->arm_configs.addItem(ArmConfig("PSM1", ArmType(ArmType::Value::PSM)));
    model2->arm_configs.addItem(ArmConfig("MTMR", ArmType(ArmType::Value::MTM_DERIVED)));
}

void MainWindow::createActions() {
    open_act = new QAction(style()->standardIcon(QStyle::SP_DirOpenIcon),
                         "&Open source", this);
    open_act->setShortcuts(QKeySequence::Open);
    open_act->setStatusTip("Open config source directory");
    connect(open_act, &QAction::triggered, this, &MainWindow::open_folder);
}

void MainWindow::createMenus() {
    file_menu = menuBar()->addMenu("&File");
    file_menu->addAction(open_act);
}

void MainWindow::open_folder() {
    QString dir = QFileDialog::getExistingDirectory(
        this,
        "Open config source",
        QDir::currentPath(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
    );

    if (dir.isEmpty()) {
        return;
    }

    config_sources->add_source(dir);
}

}
