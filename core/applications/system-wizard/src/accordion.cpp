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

#include "accordion.hpp"

namespace system_wizard {

Accordion::Accordion(QWidget* parent) : Accordion("", "", parent) { }

Accordion::Accordion(const QString& title, const QString& background_color, QWidget* parent) : QFrame(parent) {
    this->setFrameStyle(QFrame::StyledPanel);
    this->setStyleSheet("system_wizard--Accordion { background-color: " + background_color + "; border-radius: 10px; }");
    this->setSizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Maximum);

    button = new QToolButton(this);
    button->setCheckable(true); // turn into a "checkbox" button
    button->setStyleSheet("background-color: " + background_color + "; font-size: 5em; text-align: left; border: none; outline: none;");
    button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    button->setArrowType(Qt::ArrowType::RightArrow);
    button->setText(title);

    QObject::connect(button, &QPushButton::toggled, this, &Accordion::toggle);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignmentFlag::AlignTop);
    layout->setSpacing(0);
    layout->addWidget(button);

    animation = new QPropertyAnimation(this);
    animation->setPropertyName("maximumHeight");
    animation->setStartValue(0);
    animation->setDuration(200); // miiliseconcds
    animation->setEasingCurve(QEasingCurve::Type::Linear);

    setWidget(new QWidget());
}

void Accordion::setWidget(QWidget* contents) {
    layout()->removeWidget(this->contents);

    this->contents = contents;

    if (!is_open) {
        contents->setMaximumHeight(0);
    }

    layout()->addWidget(contents);
    animation->setTargetObject(this->contents);
}

void Accordion::toggle(bool button_checked) {
    is_open = button_checked;
    emit toggled(is_open);

    if (button_checked) {
        button->setArrowType(Qt::ArrowType::DownArrow);
        animation->setDirection(QAbstractAnimation::Direction::Forward);
    } else {
        button->setArrowType(Qt::ArrowType::RightArrow);
        animation->setDirection(QAbstractAnimation::Direction::Backward);
    }

    // start accordion animation
    int content_height = contents->sizeHint().height() + 10;
    animation->setEndValue(content_height);
    animation->start();
}

}
