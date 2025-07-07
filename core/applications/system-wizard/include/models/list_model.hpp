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

#include <algorithm>

#include <QtCore>
#include <json/json.h>

#include <cisstCommon/cmnPortability.h>

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
    ListModel() : QObject() {
        // make all specific change events also emit "updated" event
        QObject::connect(this, &ListModel::itemAdded,   this, &ListModel::updated);
        QObject::connect(this, &ListModel::itemUpdated, this, &ListModel::updated);
        QObject::connect(this, &ListModel::itemDeleted, this, &ListModel::updated);
        QObject::connect(this, &ListModel::reset,       this, &ListModel::updated);
    }

    virtual void updateItem(int index) {
        emit itemUpdated(index);
    }

    virtual bool canDeleteItem(int CMN_UNUSED(index)) const {
        return true;
    }

    virtual int count() const = 0;

signals:
    void itemAdded(int index);
    void itemUpdated(int index);
    void itemDeleted(int index);
    void reset();

    void updated();
};

template<typename T>
class ListModelT : public ListModel {
public:
    void appendItem(T value) {
        items.push_back(value);
        emit ListModel::itemAdded(items.size() - 1);
    }

    void deleteItem(int index) {
        if (ListModel::canDeleteItem(index)) {
            items.erase(items.begin() + index);
            emit ListModel::itemDeleted(index);
        }
    }

    const T& get(int index) const {
        return items.at(index);
    }

    T& ref(int index) {
        return items.at(index);
    }

    void clear() {
        items.clear();
        emit ListModel::reset();
    }

    void replace(std::vector<T>&& new_items) {
        items = std::move(new_items);
        emit ListModel::reset();
    }

    int count() const {
        return static_cast<int>(items.size());
    }

    Json::Value toJSON() const {
        Json::Value list(Json::arrayValue);
        for (int i = 0; i < this->count(); i++) {
            list.append(items.at(i).toJSON());
        }

        return list;
    }

    static std::unique_ptr<ListModelT<T>> fromJSON(Json::Value value) {
        if (!value.isArray()) {
            return nullptr;
        }

        auto list = std::make_unique<ListModelT<T>>();
        for (unsigned int i = 0; i < value.size(); i++) {
            std::unique_ptr<T> item_value = T::fromJSON(value[i]);
            if (item_value != nullptr) {
                list->items.push_back(*item_value);
            }
        }

        return list;
    }

private:
    std::vector<T> items;
};

// partial specialization for convienence
template<typename T>
class ListModelT<std::unique_ptr<T>> : public ListModel {
public:
    void appendItem(std::unique_ptr<T> value) {
        items.push_back(std::move(value));
        emit ListModel::itemAdded(items.size() - 1);
    }

    void deleteItem(int index) {
        if (ListModel::canDeleteItem(index)) {
            items.erase(items.begin() + index);
            emit ListModel::itemDeleted(index);
        }
    }

    const T& get(int index) const {
        return *items.at(index);
    }

    T& ref(int index) {
        return *items.at(index);
    }

    void clear() {
        items.clear();
        emit ListModel::reset();
    }

    int count() const {
        return static_cast<int>(items.size());
    }

    Json::Value toJSON() const {
        Json::Value list(Json::arrayValue);
        for (int i = 0; i < this->count(); i++) {
            list.append(items.at(i)->toJSON());
        }

        return list;
    }

    static std::unique_ptr<ListModelT<std::unique_ptr<T>>> fromJSON(Json::Value value) {
        if (!value.isArray()) {
            return nullptr;
        }

        auto list = std::make_unique<ListModelT<std::unique_ptr<T>>>();
        for (unsigned int i = 0; i < value.size(); i++) {
            std::unique_ptr<T> item_value = T::fromJSON(value[i]);
            if (item_value != nullptr) {
                list->items.push_back(std::move(item_value));
            }
        }

        return list;
    }

private:
    std::vector<std::unique_ptr<T>> items;
};

}

#endif // SYSTEM_WIZARD_LIST_MODEL
