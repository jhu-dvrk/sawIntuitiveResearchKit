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

    QString style = "system_wizard--ItemView { background-color: " + background + "; border-radius: 10px; }";
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

ListView::ListView(ListModel* model, ItemViewFactory* view_factory, SelectionMode selection_mode, bool editable)
    : editable(editable), selection_mode(selection_mode), model(model), view_factory(view_factory) {
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

    QObject::connect(model, &ListModel::itemAdded,    this, &ListView::itemAdded);
    QObject::connect(model, &ListModel::itemUpdated,  this, &ListView::itemUpdated);
    QObject::connect(model, &ListModel::itemDeleted,  this, &ListView::itemRemoved);
    QObject::connect(model, &ListModel::reset,        this, &ListView::reset);

    // populate list view if model already has data
    reset();
}

bool ListView::itemsAreInteractive() const {
    return selection_mode != SelectionMode::NONE;
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
    QLayoutItem* item = list_layout->itemAt(index);
    ItemView* item_view = qobject_cast<ItemView*>(item->widget());
    item_view->setSelected(is_selected);
    emit selected(index, is_selected);
}

void ListView::clearSelections() {
    int count = static_cast<int>(selections.size());
    for (int i = 0; i < count; i++) {
        if (selections[i]) {
            selections[i] = false;
            QLayoutItem* item = list_layout->itemAt(i);
            ItemView* item_view = qobject_cast<ItemView*>(item->widget());
            item_view->setSelected(false);
            emit selected(i, false);
        }
    }
}

void ListView::itemAdded(int id) {
    ItemView* item_view = view_factory->create(id, *this);
    list_layout->addWidget(item_view);
    selections.push_back(false);

    list_empty_display->setHidden(list_layout->count() != 0);

    Q_ASSERT(list_layout->count() == model->count());
    Q_ASSERT(selections.size() == model->count());
}

void ListView::itemUpdated(int id) {
    for (int idx = 0; idx < list_layout->count(); idx++) {
        QLayoutItem* item = list_layout->itemAt(idx);
        ItemView* item_view = qobject_cast<ItemView*>(item->widget());

        if (item_view->getId() == id) {
            item_view->updateData(id);
            Q_ASSERT(list_layout->count() == model->count());
            Q_ASSERT(selections.size() == model->count());
            return;
        }
    }

    Q_ASSERT(list_layout->count() == model->count());
    Q_ASSERT(selections.size() == model->count());
}

void ListView::itemRemoved(int id) {
    QLayoutItem* removed = list_layout->takeAt(id);
    Q_ASSERT(removed != nullptr);
    delete removed->widget();
    delete removed;

    selections.erase(selections.begin() + id);

    // shift indices of succeeding list item views
    for (int i = id; i < list_layout->count(); i++) {
        QWidget* widget = list_layout->itemAt(i)->widget();
        ItemView* item_view = qobject_cast<ItemView*>(widget);
        item_view->updateData(i);
    }

    list_empty_display->setHidden(list_layout->count() != 0);

    Q_ASSERT(list_layout->count() == model->count());
    Q_ASSERT(selections.size() == model->count());
}

void ListView::reset() {
    // added new item views if we don't have enough
    while (list_layout->count() < model->count()) {
        ItemView* item_view = view_factory->create(list_layout->count(), *this);
        list_layout->addWidget(item_view);
    }

    // remove item views if we have too many
    while (list_layout->count() > model->count()) {
        QLayoutItem* removed = list_layout->takeAt(list_layout->count() - 1);
        Q_ASSERT(removed != nullptr);
        delete removed->widget();
        delete removed;
    }

    selections = std::vector<bool>(model->count(), false);

    // update data/indices of all items to match new model data
    for (int i = 0; i < list_layout->count(); i++) {
        QWidget* widget = list_layout->itemAt(i)->widget();
        ItemView* item_view = qobject_cast<ItemView*>(widget);
        item_view->updateData(i);
        item_view->setSelected(selections[i]);
    }

    list_empty_display->setHidden(list_layout->count() != 0);

    Q_ASSERT(list_layout->count() == model->count());
    Q_ASSERT(selections.size() == model->count());
}

}
