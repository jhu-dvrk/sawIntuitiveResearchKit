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

#ifndef SYSTEM_WIZARD_LIST_VIEW
#define SYSTEM_WIZARD_LIST_VIEW

#include <QtWidgets>

#include <cisstCommon/cmnPortability.h>

#include "models/list_model.hpp"

namespace system_wizard {

enum class SelectionMode {
    NONE,
    SINGLE,
    MULTIPLE,
};

// forward declaration so ItemView can keep reference to its "parent"
class ListView;

class ItemView : public QFrame {
    Q_OBJECT

public:
    ItemView(ListView& list_view, int id, QWidget* parent = nullptr);

    int getId() const { return id; }

    virtual void updateData(int id) = 0;
    virtual void setSelected(bool selected);

protected:
    void mousePressEvent(QMouseEvent* CMN_UNUSED(event)) override;
    void mouseDoubleClickEvent(QMouseEvent* CMN_UNUSED(event)) override;

    QString base_style;
    ListView& list_view;
    int id;

    QColor base;
    QColor highlight;
};

class ItemViewFactory {
public:
    virtual ItemView* create(int id, ListView& list_view) = 0;
};

class ListView : public QWidget {
    Q_OBJECT

public:
    ListView(
        ListModel& model,
        ItemViewFactory& view_factory,
        SelectionMode selection_mode=SelectionMode::NONE,
        bool editable=false
    );

    void setEmptyMessage(std::string empty_message);

    bool itemsAreInteractive() const;

    void toggleSelection(int index);
    void selectItem(int index, bool is_selected);
    void clearSelections();

    void chooseItem(int index);
    void editItem(int index);

    const std::vector<bool>& selectedItems() const;

public slots:
    void itemAdded(int index);
    void itemUpdated(int index);
    void itemRemoved(int index);

    void reset();

signals:
    void add();
    void selected(int id, bool is_selected);
    void choose(int id);
    void edit(int id);
    void try_delete(int id);

private:
    ListModel& model;
    ItemViewFactory& view_factory;

    SelectionMode selection_mode;
    std::vector<bool> selections;

    bool editable;

    std::vector<ItemView*> item_views;
    QVBoxLayout* list_layout;
    QPushButton* add_item_button;

    QLabel* list_empty_display;
};

}

#endif // SYSTEM_WIZARD_LIST_VIEW
