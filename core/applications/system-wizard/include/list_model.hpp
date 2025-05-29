/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-28

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_LIST_MODEL
#define SYSTEM_WIZARD_LIST_MODEL

#include <QtCore>

namespace system_wizard {

/**
 * As of Qt v6.9, the MOC/meta object system doesn't work
 * with templated classes, so signals cannot be emitted
 * from a templated class.
 *
 * To allow ListModel to work with arbitrary data types,
 * it has been split into a non-template base class and
 * a templated subclass ListModelT<typename T>.
 * 
 * The base class has Q_OBJECT and can emit necessary
 * signals to any attached views when the list changes,
 * and the templated subclass can handle arbitrary data
 * types. The best of both worlds, at the cost of having
 * to split the class in two. Hopefully we'll find a better
 * way to do this, but it works reasonably well for now.
 */

class ListModel : public QObject {
    Q_OBJECT

public:
    virtual bool canDeleteItem(int index) const {
        return true;
    }

signals:
    void itemAdded(int index);
    void itemUpdated(int index);
    void itemDeleted(int index);
};

template<typename T>
class ListModelT : public ListModel {
public:
    virtual void addItem(T value) {
        items.push_back(value);
        emit itemAdded(items.size() - 1);
    }

    virtual void updateItem(int index, T value) {
        items.at(index) = value;
        emit itemUpdated(index);
    }

    virtual void deleteItem(int index) {
        if (canDeleteItem(index)) {
            items.erase(items.begin() + index);
            emit itemDeleted(index);
        }
    }

    virtual const T& get(int index) const {
        return items.at(index);
    }

private:
    std::vector<T> items;
};

}

#endif // SYSTEM_WIZARD_LIST_MODEL
