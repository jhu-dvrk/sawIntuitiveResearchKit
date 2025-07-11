/*
  Author(s):  Brendan Burkhart
  Created on: 2025-06-08

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "editor.hpp"

#include "config_editor.hpp"
#include "models/config_model.hpp"

namespace system_wizard {

Editor::Editor(ConfigSources& config_sources, QWidget* parent)
    : QStackedWidget(parent),
      config_sources(&config_sources) {
    tabs = new QTabWidget();
    tabs->setMovable(true);
    tabs->setTabsClosable(true);

    this->addWidget(tabs);

    QToolButton *add_config_button = new QToolButton();
    add_config_button->setText("+");
    QLabel* no_tabs_view = new QLabel("Add config by pressing \"+\"");
    no_tabs_view->setAlignment(Qt::AlignCenter);
    no_tabs_view->setMargin(10);
    // Add empty, not enabled dummy tab
    tabs->addTab(no_tabs_view, QString());
    tabs->setTabEnabled(0, false);
    // Add tab button to dummy tab
    tabs->tabBar()->setTabButton(0, QTabBar::RightSide, add_config_button);

    QObject::connect(add_config_button, &QToolButton::clicked, this, &Editor::newConfig);

    // In Qt < 6, default Close key sequence is Ctrl+F4 on Windows but Ctrl+W is listed as alternative
    // In Qt 6, default is Ctrl+W which I like more so if available default to only using that
    const QKeySequence standard_close(Qt::CTRL | Qt::Key_W);
    if (QKeySequence::keyBindings(QKeySequence::Close).contains(standard_close)) {
        close_config_shortcut = new QShortcut(standard_close, this);
    } else {
        close_config_shortcut = new QShortcut(QKeySequence::Close, this);
    }

    QObject::connect(close_config_shortcut, &QShortcut::activated, this, [this](){ closeConfig(tabs->currentIndex()); });
    QObject::connect(tabs->tabBar(), &QTabBar::tabCloseRequested, this, [this](int index){ closeConfig(index); });

    newConfig();
}

void Editor::newConfig() {
    std::unique_ptr<SystemConfigModel> model = std::make_unique<SystemConfigModel>();
    model->io_configs->appendItem(IOConfig("io")); // default I/O

    std::unique_ptr<ConsoleConfig> default_console = std::make_unique<ConsoleConfig>();
    default_console->name = "Console";
    model->console_configs->appendItem(std::move(default_console));

    std::unique_ptr<ConfigEditor> editor = std::make_unique<ConfigEditor>(std::move(model), *config_sources);
    createTab(std::move(editor));
}

void Editor::openConfig() {
    QString file_name = QFileDialog::getOpenFileName();
    if (file_name.isEmpty()) {
        return;
    }

    std::filesystem::path file_path(file_name.toStdString());
    std::unique_ptr<ConfigEditor> editor = ConfigEditor::open(file_path, *config_sources);
    if (editor == nullptr) {
        // TODO: display validation/error message
        return;
    }

    createTab(std::move(editor));
}

void Editor::save() {
    if (tabs->currentIndex() <= 0) {
        return;
    }

    ConfigEditor* editor = qobject_cast<ConfigEditor*>(tabs->currentWidget());
    editor->save();
}

void Editor::saveAs() {
    if (tabs->currentIndex() <= 0) {
        return;
    }

    ConfigEditor* editor = qobject_cast<ConfigEditor*>(tabs->currentWidget());
    editor->saveAs();
}

void Editor::closeConfig(int index) {
    if (index <= 0) {
        return;
    }

    ConfigEditor* editor = qobject_cast<ConfigEditor*>(tabs->widget(index));
    bool ok_to_close = editor->close();
    if (ok_to_close) {
        tabs->removeTab(index);
        delete editor;
    }
}

void Editor::createTab(std::unique_ptr<ConfigEditor> config_editor) {
    ConfigEditor* ptr = config_editor.get(); // save non-owning pointer
    tabs->addTab(config_editor.release(), ""); // transfer ownership to Qt GUI tree

    auto update_tab_title = [this, ptr]() {
        std::optional<std::filesystem::path> path = ptr->savePath();
        std::string filename = path ? path->filename().string() : "Untitled config";
        bool unsaved_changes = !ptr->changesSaved();
        std::string label = unsaved_changes ? filename + "*" : filename;

        // get tab index in case it has changed (earlier tab closed or tabs re-arranged)
        int index = tabs->indexOf(ptr);
        tabs->setTabText(index, QString::fromStdString(label));
    };

    QObject::connect(ptr, &ConfigEditor::savePathChanged, this, update_tab_title);
    QObject::connect(ptr, &ConfigEditor::saveStateChanged, this, update_tab_title);

    update_tab_title();
    tabs->setCurrentIndex(tabs->count() - 1);
}

}
