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
    setStyleSheet("background-color: palette(base); border-radius: 10px;");
}

ListView::ListView(ListModel* model, ItemViewFactory* view_factory)
    : model(model), view_factory(view_factory) {
    QVBoxLayout* layout = new QVBoxLayout(this);

    QHBoxLayout* add_item_layout = new QHBoxLayout();
    add_item_button = new QPushButton("Add");
    add_item_layout->addWidget(add_item_button);
    add_item_layout->addStretch();
    layout->addLayout(add_item_layout);

    list_layout = new QVBoxLayout();
    layout->addLayout(list_layout);

    QObject::connect(add_item_button, &QPushButton::clicked, this, &ListView::add);

    QObject::connect(model, &ListModel::itemAdded,   this, &ListView::itemAdded);
    QObject::connect(model, &ListModel::itemUpdated, this, &ListView::itemUpdated);
    QObject::connect(model, &ListModel::itemDeleted, this, &ListView::itemRemoved);
}

void ListView::itemAdded(int id) {
    ItemView* item_view = view_factory->create(id, *this);
    list_layout->addWidget(item_view);
}

void ListView::itemUpdated(int id) {
    for (int idx = 0; idx < list_layout->count(); idx++) {
        QLayoutItem* item = list_layout->itemAt(idx);
        ItemView* item_view = qobject_cast<ItemView*>(item->widget());

        if (item_view->getId() == id) {
            item_view->updateData(id);
            return;
        }
    }
}

void ListView::itemRemoved(int id) {
    QLayoutItem* removed = list_layout->takeAt(id);
    Q_ASSERT(removed != nullptr);
    delete removed->widget();
    delete removed;

    // shift indices of succeeding list item views
    for (int i = id; i < list_layout->count(); i++) {
        QWidget* widget = list_layout->itemAt(i)->widget();
        ItemView* item_view = qobject_cast<ItemView*>(widget);
        item_view->updateData(id);
    }
}

}
