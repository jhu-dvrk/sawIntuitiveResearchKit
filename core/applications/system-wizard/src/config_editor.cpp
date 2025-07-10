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

#include "accordion.hpp"
#include "arm_view.hpp"

#include <QTreeWidget>

namespace system_wizard {

ConfigEditor::ConfigEditor(std::unique_ptr<SystemConfigModel> config_model, ConfigSources& config_sources, QWidget* parent)
    : QWidget(parent),
      model(std::move(config_model)),
      changes_saved(true),
      io_editor(*model, this),
      arm_editor(*model, config_sources, this)  {

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
    Accordion* console_panel = new Accordion("Consoles", "SteelBlue", has_consoles, this);
    ConsolesContainer* consoles = new ConsolesContainer(*model);
    console_panel->setWidget(consoles);

    scroller_layout->addWidget(ios);
    scroller_layout->addWidget(arms);
    scroller_layout->addWidget(console_panel);
    scroller_layout->addStretch();
}

std::unique_ptr<ConfigEditor> ConfigEditor::open(std::filesystem::path config_file, ConfigSources& sources) {
    auto model = SystemConfigModel::load(config_file);
    if (model == nullptr) {
        return nullptr;
    }
    auto editor = std::make_unique<ConfigEditor>(std::move(model), sources);
    editor->changes_saved = true;
    editor->setSavePath(config_file);
    return editor;
}

void ConfigEditor::setSavePath(std::filesystem::path path) {
    this->save_path = path;
    const QString qpath = QString::fromStdString(path.string());
    path_display->setText(qpath);
    emit savePathChanged(qpath);
}

bool ConfigEditor::save() {
    if (!save_path.has_value()) {
        QString file_name = QFileDialog::getSaveFileName();
        if (file_name.isEmpty()) {
            return false;
        }

        std::filesystem::path file_path(file_name.toStdString());
        setSavePath(file_path);
    }

    changes_saved = model->save(save_path.value());
    emit saveStateChanged(changes_saved);
    return true;
}

bool ConfigEditor::saveAs() {
    QString file_name = QFileDialog::getSaveFileName();
    if (file_name.isEmpty()) {
        return false;
    }

    std::filesystem::path file_path(file_name.toStdString());
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