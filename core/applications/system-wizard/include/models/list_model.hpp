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
#include <optional>

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
        QObject::connect(this, &ListModel::itemAdded,   this, &ListModel::updated);
        QObject::connect(this, &ListModel::itemUpdated, this, &ListModel::updated);
        QObject::connect(this, &ListModel::itemDeleted, this, &ListModel::updated);
        QObject::connect(this, &ListModel::reset,       this, &ListModel::updated);
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
    virtual void addItem(T value) = 0;
    virtual void updateItem(int index, T value) = 0;
    virtual void deleteItem(int index) = 0;

    virtual const T& get(int index) const = 0;

    virtual void clear() = 0;
    virtual void update(std::vector<T>& new_items) = 0;

    template<typename U=T>
    Json::Value toJSON() const {
        Json::Value list(Json::arrayValue);
        for (int i = 0; i < this->count(); i++) {
            list.append(this->get(i).toJSON());
        }

        return list;
    }
};

template<typename T>
class VectorList : public ListModelT<T> {
public:
    void addItem(T value) override {
        items.push_back(value);
        emit ListModel::itemAdded(items.size() - 1);
    }

    void updateItem(int index, T value) override {
        items.at(index) = value;
        emit ListModel::itemUpdated(index);
    }

    void deleteItem(int index) override {
        if (ListModel::canDeleteItem(index)) {
            items.erase(items.begin() + index);
            emit ListModel::itemDeleted(index);
        }
    }

    const T& get(int index) const override {
        return items.at(index);
    }

    void clear() override {
        items.clear();
        emit ListModel::reset();
    }

    void update(std::vector<T>& new_items) override {
        items = new_items;
        emit ListModel::reset();
    }

    int count() const override {
        return static_cast<int>(items.size());
    }

    template<typename U=T>
    static std::unique_ptr<VectorList<U>> fromJSON(Json::Value value) {
        if (!value.isArray()) {
            return nullptr;
        }

        auto list = std::make_unique<VectorList<U>>();
        for (unsigned int i = 0; i < value.size(); i++) {
            list->items.push_back(U::fromJSON(value[i]));
        }

        return list;
    }

private:
    std::vector<T> items;
};

}

#endif // SYSTEM_WIZARD_LIST_MODEL
