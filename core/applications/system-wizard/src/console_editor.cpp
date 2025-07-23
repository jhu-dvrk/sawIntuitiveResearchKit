#include "accordion.hpp"
#include "console_editor.hpp"
#include "console_inputs_editor.hpp"
#include "teleop_view.hpp"

namespace system_wizard {

ConsoleEditor::ConsoleEditor(ConsoleConfig& config, SystemConfigModel& model, QWidget* parent) :
    QWidget(parent),
    config(&config),
    inputs_editor(new ConsoleInputsEditor(*config.inputs, *model.arm_configs)),
    psm_teleop_editor(*config.psm_teleops, TeleopType::Value::PSM_TELEOP, *model.arm_configs),
    ecm_teleop_editor(*config.ecm_teleops, TeleopType::Value::ECM_TELEOP, *model.arm_configs)
{
    config.provideSources(*model.arm_configs);

    QVBoxLayout* layout = new QVBoxLayout(this);

    QHBoxLayout* header = new QHBoxLayout();
    name_input = new QLineEdit(QString::fromStdString(config.name));
    QRegularExpression name_regex("[a-zA-Z]\\w*"); // must start with letter, followed by 0+ letters/digits
    QValidator *name_validator = new QRegularExpressionValidator(name_regex, this);
    name_input->setValidator(name_validator);
    header->addWidget(new QLabel("Console name:"));
    header->addWidget(name_input);
    header->addStretch();
    layout->addLayout(header);

    QObject::connect(name_input, &QLineEdit::textChanged, this, [this](QString s){
        emit nameChanged(s.toStdString());
        this->config->name = s.toStdString();
        emit this->config->updated();
    });

    QFrame *header_separator = new QFrame();
    header_separator->setFrameShape(QFrame::HLine);
    header_separator->setFrameShadow(QFrame::Sunken);
    layout->addWidget(header_separator);

    layout->addWidget(inputs_editor);

    QFrame *teleops_separator = new QFrame();
    teleops_separator->setFrameShape(QFrame::HLine);
    teleops_separator->setFrameShadow(QFrame::Sunken);
    layout->addWidget(teleops_separator);

    Accordion* psm_teleops = new Accordion("PSM Teleops", "rgb(79, 146, 201)", true);
    auto psm_teleop_factory = [&config](int index, ListView& view) {
        return std::make_unique<PSMTeleopView>(config, view, index);
    };
    ListView* psm_teleop_list = new ListView(*config.psm_teleops, psm_teleop_factory, SelectionMode::NONE, true);
    psm_teleop_list->setEmptyMessage("No PSM teleops added - teleoperation mode will not be available");
    psm_teleops->setWidget(psm_teleop_list);

    QObject::connect(psm_teleop_list, &ListView::add, this, [this]() { psm_teleop_editor.setId(-1); psm_teleop_editor.open(); });
    QObject::connect(psm_teleop_list, &ListView::try_delete, config.psm_teleops.get(), &ListModelT<TeleopConfig>::deleteItem);
    QObject::connect(psm_teleop_list, &ListView::choose, this, [this](int id) { psm_teleop_editor.setId(id); psm_teleop_editor.open(); });
    QObject::connect(psm_teleop_list, &ListView::edit, this, [this](int id) { psm_teleop_editor.setId(id); psm_teleop_editor.open(); });

    Accordion* ecm_teleops = new Accordion("ECM Teleops", "rgb(79, 146, 201)", true);
    auto ecm_teleop_factory = [&config](int index, ListView& view) {
        return std::make_unique<ECMTeleopView>(config, view, index);
    };
    ListView* ecm_teleop_list = new ListView(*config.ecm_teleops, ecm_teleop_factory, SelectionMode::NONE, true);
    ecm_teleop_list->setEmptyMessage("No ECM teleops added - camera movement will not be available");
    ecm_teleops->setWidget(ecm_teleop_list);

    QObject::connect(ecm_teleop_list, &ListView::add, this, [this]() { ecm_teleop_editor.setId(-1); ecm_teleop_editor.open(); });
    QObject::connect(ecm_teleop_list, &ListView::try_delete, config.ecm_teleops.get(), &ListModelT<TeleopConfig>::deleteItem);
    QObject::connect(ecm_teleop_list, &ListView::choose, this, [this](int id) { ecm_teleop_editor.setId(id); ecm_teleop_editor.open(); });
    QObject::connect(ecm_teleop_list, &ListView::edit, this, [this](int id) { ecm_teleop_editor.setId(id); ecm_teleop_editor.open(); });

    layout->addWidget(psm_teleops);
    layout->addWidget(ecm_teleops);
}

bool ConsoleEditor::close() {
    QMessageBox message;
    message.setText("Are you sure you want to delete this console?");
    message.setStandardButtons(QMessageBox::Discard | QMessageBox::Cancel);
    message.setDefaultButton(QMessageBox::Cancel);
    int ret = message.exec();

    switch (ret) {
    case QMessageBox::Discard:
        return true;
    case QMessageBox::Cancel:
        return false;
    default:
        Q_ASSERT(false);
        return false;
    }
}

ConsolesContainer::ConsolesContainer(SystemConfigModel& model, QWidget* parent) : QTabWidget(parent), model(&model) {
    this->setMovable(true);
    this->setTabsClosable(true);

    QToolButton *add_console_button = new QToolButton();
    add_console_button->setText("+");
    // Add empty, not enabled dummy tab
    QLabel* no_tabs_view = new QLabel("Add surgeon console by pressing \"+\"");
    no_tabs_view->setAlignment(Qt::AlignCenter);
    no_tabs_view->setMargin(10);
    this->addTab(no_tabs_view, QString());
    this->setTabEnabled(0, false);
    // Attach add console button to dummy tab
    this->tabBar()->setTabButton(0, QTabBar::RightSide, add_console_button);

    QObject::connect(add_console_button, &QToolButton::clicked, this, &ConsolesContainer::addConsole);
    QObject::connect(this->tabBar(), &QTabBar::tabCloseRequested, this, [this](int index){ removeConsole(index); });

    for (int idx = 0; idx < model.console_configs->count(); idx++) {
        openConsole(model.console_configs->ref(idx));
    }
}

void ConsolesContainer::addConsole() {
    std::unique_ptr<ConsoleConfig> config = std::make_unique<ConsoleConfig>();
    config->name = "Console";
    ConsoleConfig* config_ptr = config.get();
    model->console_configs->appendItem(std::move(config));

    openConsole(*config_ptr);
}

void ConsolesContainer::openConsole(ConsoleConfig& config) {
    std::unique_ptr<ConsoleEditor> editor = std::make_unique<ConsoleEditor>(config, *model);
    ConsoleEditor* ptr = editor.get(); // save non-owning pointer
    this->addTab(editor.release(), QString::fromStdString(config.name)); // transfer ownership to Qt GUI tree

    auto update_tab_title = [this, ptr](std::string name) {
        // get tab index in case it has changed (earlier tab closed or tabs re-arranged)
        int index = this->indexOf(ptr);
        this->setTabText(index, QString::fromStdString(name));
    };

    QObject::connect(ptr, &ConsoleEditor::nameChanged, this, update_tab_title);

    this->setCurrentIndex(this->count() - 1);
}

void ConsolesContainer::removeConsole(int index) {
    if (index <= 0) {
        return;
    }

    ConsoleEditor* editor = qobject_cast<ConsoleEditor*>(this->widget(index));
    bool ok_to_close = editor->close();
    if (ok_to_close) {
        model->console_configs->deleteItem(index);
        this->removeTab(index);
        delete editor;
    }
}

}
