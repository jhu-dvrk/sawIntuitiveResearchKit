/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-27

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "list_view.hpp"

namespace system_wizard {

ItemView::ItemView(ListView& list_view, int id, QWidget* parent)
    : QFrame(parent), list_view(list_view), id(id) {
    setFrameStyle(QFrame::StyledPanel);

    base = palette().color(QPalette::Base);
    highlight = palette().color(QPalette::Highlight);

    setSelected(false);
}

void ItemView::setSelected(bool selected) {
    float blend = selected ? 0.25 : 0.75;

    QColor hover_color = QColor::fromRgbF(
        blend*base.redF()   + (1.0 - blend)*highlight.redF(),
        blend*base.greenF() + (1.0 - blend)*highlight.greenF(),
        blend*base.blueF()  + (1.0 - blend)*highlight.blueF()
    );

    QString background = selected ? "palette(highlight)" : "palette(base)";
    QString text = selected ? "palette(highlighted-text)" : "palette(text)";

    QString style = "QLabel { color: " + text + ";  } system_wizard--ItemView { background-color: " + background + "; border-radius: 10px; }";
    QString hover_style_template = QString("system_wizard--ItemView:hover { background-color: #%1; }");
    // convert color to base-16 string with 0-width padding
    QString hover_style = hover_style_template.arg(hover_color.rgba(), 0, 16);

    if (list_view.itemsAreInteractive()) {
        style += " " + hover_style;
    }

    setStyleSheet(style);
}

void ItemView::mousePressEvent(QMouseEvent* CMN_UNUSED(event)) {
    list_view.toggleSelection(id);
}

void ItemView::mouseDoubleClickEvent(QMouseEvent* CMN_UNUSED(event)) {
    list_view.chooseItem(id);
}

ListView::ListView(ListModel& model, Factory view_factory, SelectionMode selection_mode, bool editable)
    : model(model), view_factory(view_factory), selection_mode(selection_mode), editable(editable) {
    QVBoxLayout* layout = new QVBoxLayout(this);

    if (editable) {
        QHBoxLayout* add_item_layout = new QHBoxLayout();
        add_item_button = new QPushButton("Add");
        add_item_layout->addWidget(add_item_button);
        add_item_layout->addStretch();
        layout->addLayout(add_item_layout);

        QObject::connect(add_item_button, &QPushButton::clicked, this, &ListView::add);
    }

    list_empty_display = new QLabel("No items");
    layout->addWidget(list_empty_display);

    list_layout = new QVBoxLayout();
    layout->addLayout(list_layout);

    QObject::connect(&model, &ListModel::itemAdded,    this, &ListView::itemAdded);
    QObject::connect(&model, &ListModel::itemUpdated,  this, &ListView::itemUpdated);
    QObject::connect(&model, &ListModel::itemDeleted,  this, &ListView::itemRemoved);
    QObject::connect(&model, &ListModel::reset,        this, &ListView::reset);

    // populate list view if model already has data
    reset();
}

bool ListView::itemsAreInteractive() const {
    return editable || selection_mode != SelectionMode::NONE;
}

void ListView::setEmptyMessage(std::string empty_message) {
    list_empty_display->setText(QString::fromStdString(empty_message));
}

void ListView::toggleSelection(int index) {
    selectItem(index, !selections.at(index));
}

void ListView::selectItem(int index, bool is_selected) {
    if (selection_mode == SelectionMode::NONE) {
        return;
    }

    if (is_selected == selections.at(index)) {
        return;
    }

    if (is_selected && selection_mode != SelectionMode::MULTIPLE) {
        clearSelections();
    }

    selections.at(index) = is_selected;
    item_views.at(index)->setSelected(is_selected);
    emit selected(index, is_selected);
}

void ListView::clearSelections() {
    int count = static_cast<int>(selections.size());
    for (int i = 0; i < count; i++) {
        if (selections[i]) {
            selections[i] = false;
            item_views.at(i)->setSelected(false);
            emit selected(i, false);
        }
    }
}

void ListView::chooseItem(int index) {
    emit choose(index);
}

void ListView::editItem(int index) {
    emit edit(index);
}

const std::vector<bool>& ListView::selectedItems() const {
    return selections;
}

void ListView::itemAdded(int id) {
    std::unique_ptr<ItemView> item_view = view_factory(id, *this);
    item_views.push_back(item_view.get());
    list_layout->addWidget(item_view.release());
    selections.push_back(false);

    list_empty_display->setHidden(list_layout->count() != 0);

    Q_ASSERT(list_layout->count() == model.count());
    Q_ASSERT(selections.size() == model.count());
    Q_ASSERT(item_views.size() == model.count());
}

void ListView::itemUpdated(int id) {
    for (int idx = 0; idx < list_layout->count(); idx++) {
        ItemView* item_view = item_views.at(idx);
        if (item_view->getId() == id) {
            item_view->updateData(id);
            Q_ASSERT(list_layout->count() == model.count());
            Q_ASSERT(selections.size() == model.count());
            Q_ASSERT(item_views.size() == model.count());
            return;
        }
    }

    Q_ASSERT(list_layout->count() == model.count());
    Q_ASSERT(selections.size() == model.count());
    Q_ASSERT(item_views.size() == model.count());
}

void ListView::itemRemoved(int id) {
    selections.erase(selections.begin() + id);
    item_views.erase(item_views.begin() + id);

    QLayoutItem* removed = list_layout->takeAt(id);
    Q_ASSERT(removed != nullptr);
    delete removed->widget();
    delete removed;

    // shift indices of succeeding list item views
    for (int i = id; i < list_layout->count(); i++) {
        item_views.at(i)->updateData(i);
    }

    list_empty_display->setHidden(list_layout->count() != 0);

    Q_ASSERT(list_layout->count() == model.count());
    Q_ASSERT(selections.size() == model.count());
    Q_ASSERT(item_views.size() == model.count());
}

void ListView::reset() {
    // added new item views if we don't have enough
    while (list_layout->count() < model.count()) {
        std::unique_ptr<ItemView> item_view = view_factory(list_layout->count(), *this);
        item_views.push_back(item_view.get());
        list_layout->addWidget(item_view.release());
    }

    // remove item views if we have too many
    while (list_layout->count() > model.count()) {
        item_views.erase(item_views.end() - 1);
        QLayoutItem* removed = list_layout->takeAt(list_layout->count() - 1);
        Q_ASSERT(removed != nullptr);
        delete removed->widget();
        delete removed;
    }

    selections = std::vector<bool>(model.count(), false);

    // update data/indices of all items to match new model data
    for (int i = 0; i < list_layout->count(); i++) {
        item_views.at(i)->updateData(i);
        item_views.at(i)->setSelected(selections[i]);
    }

    list_empty_display->setHidden(list_layout->count() != 0);

    Q_ASSERT(list_layout->count() == model.count());
    Q_ASSERT(selections.size() == model.count());
    Q_ASSERT(item_views.size() == model.count());
}

}
