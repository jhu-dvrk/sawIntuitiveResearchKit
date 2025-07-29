/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-25

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef CONFIG_WIZARD_ENUM_LIST_MODEL
#define CONFIG_WIZARD_ENUM_LIST_MODEL

#include <QtWidgets>

#include <cisstCommon/cmnPortability.h>

namespace config_wizard {

template <typename T>
class EnumListModel : public QAbstractItemModel {
public:
    QModelIndex index(int row, int column, const QModelIndex &CMN_UNUSED(parent) = QModelIndex()) const {
        return createIndex(row, column, row);
    }

    QModelIndex parent(const QModelIndex &CMN_UNUSED(index)) const {
        return QModelIndex(); // invalid index since no item has a parent
    }

    int rowCount(const QModelIndex &CMN_UNUSED(parent) = QModelIndex()) const {
        return T::count();
    }

    int columnCount(const QModelIndex &CMN_UNUSED(parent) = QModelIndex()) const {
        return 1;
    }

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const {
        T t(typename T::Value(index.row()));

        switch (role) {
        case Qt::DisplayRole:
            return QString::fromStdString(t.name());
        case Qt::ToolTipRole:
        case Qt::WhatsThisRole:
            return QString::fromStdString(t.explain());
        default:
            return QVariant();
        }
    }
};

}

#endif // CONFIG_WIZARD_ENUM_LIST_MODEL
