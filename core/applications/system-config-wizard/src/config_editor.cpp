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

#include "config_editor.hpp"

#include <QTreeWidget>
#include <QtCore>
#include <filesystem>
#include <memory>
#include <qerrormessage.h>
#include <qmessagebox.h>

#include "accordion.hpp"
#include "arm_view.hpp"
#include "io_view.hpp"
#include "system_launcher.hpp"

namespace config_wizard {

ConfigEditor::ConfigEditor(
    std::unique_ptr<SystemConfigModel> config_model,
    ConfigSources& config_sources,
    SystemLauncher& launcher,
    QWidget* parent
)
    : QWidget(parent),
      model(std::move(config_model)),
      changes_saved(true),
      io_editor(*model, this),
      arm_editor(*model, config_sources, this),
      sources(&config_sources),
      launcher(&launcher) {

    // treat any update to model as resulting in unsaved changes
    QObject::connect(model.get(), &SystemConfigModel::updated, this, [this](){
        changes_saved = false;
        emit saveStateChanged(changes_saved);
    });

    QVBoxLayout* layout = new QVBoxLayout(this);

    QVBoxLayout* header = new QVBoxLayout();
    layout->addLayout(header);

    QHBoxLayout* file_path_layout = new QHBoxLayout();
    path_display = new QLabel();
    file_path_layout->addWidget(path_display);
    file_path_layout->addStretch();
    header->addLayout(file_path_layout);
    path_display->setVisible(false);

    QColor base = palette().color(QPalette::Base);
    QColor primary = QColor("mediumseagreen");

    float blend = 0.15;

    QColor hover_color = QColor::fromRgbF(
        blend*base.redF()   + (1.0 - blend)*primary.redF(),
        blend*base.greenF() + (1.0 - blend)*primary.greenF(),
        blend*base.blueF()  + (1.0 - blend)*primary.blueF()
    );

    blend = 0.75;
    QColor disabled_color = QColor::fromRgbF(
        blend*base.redF()   + (1.0 - blend)*primary.redF(),
        blend*base.greenF() + (1.0 - blend)*primary.greenF(),
        blend*base.blueF()  + (1.0 - blend)*primary.blueF()
    );

    QString style = "QPushButton { background-color: mediumseagreen; border-radius: 6px; padding: 0.25em; font-size: 12pt; text-align: left; border: none; outline: none; }";
    QString hover_style_template = QString("QPushButton:hover { background-color: #%1; }");
    // convert color to base-16 string with 0-width padding
    QString hover_style = hover_style_template.arg(hover_color.rgba(), 0, 16);
    QString disabled_style_template = QString("QPushButton:disabled { background-color: #%1; }");
    // convert color to base-16 string with 0-width padding
    QString disabled_style = disabled_style_template.arg(disabled_color.rgba(), 0, 16);

    QHBoxLayout* launch_layout = new QHBoxLayout();
    QIcon launch_icon = this->style()->standardIcon(QStyle::SP_MediaPlay);

    launch_button = new QPushButton(launch_icon, "Launch system");
    launch_button->setStyleSheet(style + hover_style + disabled_style);
    launch_layout->addWidget(launch_button);
    launch_layout->addStretch();
    header->addLayout(launch_layout);
    QObject::connect(launch_button, &QPushButton::pressed, this, [this]() {
        std::string error_message = this->launcher->launch(*this->model);
        if (!error_message.empty()) {
            QMessageBox::critical(this, "dVRK System Error", QString::fromStdString(error_message));
        }
    });
    QObject::connect(&launcher, &SystemLauncher::stateChanged, this, &ConfigEditor::updateLaunchButton);
    updateLaunchButton(); // make sure we disable if editor is opened while system is already running

    QScrollArea* scroller = new QScrollArea();
    QWidget* scroller_contents = new QWidget();
    QVBoxLayout* scroller_layout = new QVBoxLayout(scroller_contents);
    // per docs, must add layout before adding to scroll area
    scroller->setWidget(scroller_contents);
    scroller->setWidgetResizable(true);
    layout->addWidget(scroller);

    // I/O config panel

    Accordion* ios = new Accordion("I/O Connections", "Salmon", true, this);
    auto io_factory = [this](int index, ListView& list) {
        return std::make_unique<IOView>(*this->model, list, index);
    };
    ListView* io_list = new ListView(*model->io_configs, io_factory, SelectionMode::NONE, true);
    ios->setWidget(io_list);

    QObject::connect(io_list, &ListView::add, this, [this]() { io_editor.setId(-1); io_editor.open(); });
    QObject::connect(io_list, &ListView::choose, this, [this](int id) { io_editor.setId(id); io_editor.open(); });
    QObject::connect(io_list, &ListView::edit, this, [this](int id) { io_editor.setId(id); io_editor.open(); });
    QObject::connect(io_list, &ListView::try_delete, model->io_configs.get(), &ListModelT<IOConfig>::deleteItem);

    // Arm config panel

    Accordion* arms = new Accordion("Arms", "LightSeaGreen", true, this);
    auto arm_factory = [this](int index, ListView& list) {
        return std::make_unique<ArmView>(*this->model, list, index);
    };
    ListView* arm_list = new ListView(*model->arm_configs, arm_factory, SelectionMode::NONE, true);
    arm_list->setEmptyMessage("No arms configured yet");
    arms->setWidget(arm_list);

    QObject::connect(arm_list, &ListView::add, this, [this]() { arm_editor.setId(-1); arm_editor.open(); });
    QObject::connect(arm_list, &ListView::try_delete, model->arm_configs.get(), &ListModelT<ArmConfig>::deleteItem);
    QObject::connect(arm_list, &ListView::choose, this, [this](int id) { arm_editor.setId(id); arm_editor.open(); });
    QObject::connect(arm_list, &ListView::edit, this, [this](int id) { arm_editor.setId(id); arm_editor.open(); });

    // Console config panel

    bool has_consoles = model->console_configs->count() > 0;
    Accordion* console_panel = new Accordion("Consoles", "rgb(79, 146, 201)", has_consoles, this);
    ConsolesContainer* consoles = new ConsolesContainer(*model);
    console_panel->setWidget(consoles);

    scroller_layout->addWidget(ios);
    scroller_layout->addWidget(arms);
    scroller_layout->addWidget(console_panel);
    scroller_layout->addStretch();
}

std::unique_ptr<ConfigEditor> ConfigEditor::open(std::filesystem::path config_file, ConfigSources& sources, SystemLauncher& launcher) {
    auto model = SystemConfigModel::load(config_file);
    if (model == nullptr) {
        return nullptr;
    }
    auto editor = std::make_unique<ConfigEditor>(std::move(model), sources, launcher);
    editor->changes_saved = true;
    editor->setSavePath(config_file);
    return editor;
}

void ConfigEditor::setSavePath(std::filesystem::path path) {
    path = std::filesystem::weakly_canonical(path);
    this->save_path = path;
    const QString qpath = QString::fromStdString(path.string());
    path_display->setText(qpath);
    path_display->setVisible(!path.empty());
    emit savePathChanged(qpath);
}

void ConfigEditor::updateLaunchButton() {
    launch_button->setDisabled(launcher->isRunning());
    if (launcher->isRunning()) {
        launch_button->setToolTip("dVRK is already running");
    } else {
        launch_button->setToolTip("Launch the dVRK with this system configuration");
    }
}

bool ConfigEditor::save() {
    if (!save_path.has_value()) {
        return saveAs();
    }

    changes_saved = model->save(save_path.value());
    emit saveStateChanged(changes_saved);
    return true;
}

bool ConfigEditor::saveAs() {
    QString dir = QString();
    // open file picker in same location as current file
    if (save_path.has_value()) {
        dir = QString::fromStdString(save_path.value().parent_path());
    // or in the source directory if config is unsaved
    } else {
        auto source_dir = sources->dir();
        if (source_dir.has_value()) {
            dir = QString::fromStdString(source_dir.value());
        }
    }

    QString file_name = QFileDialog::getSaveFileName(this, "Save system config", dir, "Config file (*.json)");
    if (file_name.isEmpty()) {
        return false;
    }

    // Ensure we save as .json, e.g. system-PSM1.json if user just types system-PSM1
    std::filesystem::path file_path(file_name.toStdString());
    if (file_path.extension() != ".json") {
        file_path += ".json";
    }

    setSavePath(file_path);
    changes_saved = model->save(save_path.value());
    emit saveStateChanged(changes_saved);
    return true;
}

bool ConfigEditor::close() {
    if (changes_saved) {
        return true;
    }

    QMessageBox message;
    message.setText("Do you want to save your changes?");
    message.setInformativeText("Your changes will be lost if you do not save");
    message.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    message.setDefaultButton(QMessageBox::Save);
    int ret = message.exec();

    switch (ret) {
    case QMessageBox::Save:
        return save();
    case QMessageBox::Discard:
        return true;
    case QMessageBox::Cancel:
        return false;
    default:
        Q_ASSERT(false);
        return false;
    }
}

}