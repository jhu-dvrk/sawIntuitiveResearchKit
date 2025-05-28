/*
  Author(s):  Brendan Burkhart
  Created on: 2025-05-24

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef SYSTEM_WIZARD_ACCORDION
#define SYSTEM_WIZARD_ACCORDION

#include <QtWidgets>

namespace system_wizard {

class Accordion : public QFrame {
    Q_OBJECT

public:
    Accordion(QWidget* parent = nullptr);
    Accordion(const QString& title, const QString& background_color, QWidget* parent = nullptr);

    void setWidget(QWidget* contents);

signals:
    void toggled(bool is_open);

private:
    void toggle(bool button_checked);
    void animation_finished();

    bool is_open;

    QToolButton* button;
    QWidget* contents;

    QPropertyAnimation* animation;
};

}

#endif // SYSTEM_WIZARD_ACCORDION
