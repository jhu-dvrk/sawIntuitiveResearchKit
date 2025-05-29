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

#include "list_model.hpp"

#include <functional>

#include <QtWidgets>

namespace system_wizard {

// forward declaration so ItemView can keep reference to its "parent"
class ListView;

class ItemView : public QFrame {
    Q_OBJECT

public:
    ItemView(ListView& list_view, int id, QWidget* parent = nullptr);

    int getId() const { return id; }

public slots:
    virtual void updateData(int id) = 0;

protected:
    ListView& list_view;
    int id;
};

class ItemViewFactory {
public:
    virtual ItemView* create(int id, ListView& list_view) = 0;
};

class ListView : public QWidget {
    Q_OBJECT

public:
    ListView(ListModel* model, ItemViewFactory* view_factory);

public slots:
    void itemAdded(int index);
    void itemUpdated(int index);
    void itemRemoved(int index);

signals:
    void add();
    void edit(int id);
    void try_delete(int id);

private:
    ListModel* model;
    ItemViewFactory* view_factory;

    QVBoxLayout* list_layout;
    QPushButton* add_item_button;
};

}

#endif // SYSTEM_WIZARD_LIST_VIEW
