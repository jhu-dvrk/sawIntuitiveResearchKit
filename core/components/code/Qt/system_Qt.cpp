/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-13

  (C) Copyright 2015-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/system_Qt.h>

// cisst/saw
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>
#include <cisstMultiTask/mtsIntervalStatisticsQtWidget.h>

#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawControllers/mtsPIDQtWidget.h>

#include <sawIntuitiveResearchKit/system.h>
#include <sawIntuitiveResearchKit/arm_proxy.h>
#include <sawIntuitiveResearchKit/teleop_proxy.h>
#include <sawIntuitiveResearchKit/console.h>
#include <sawIntuitiveResearchKit/mtsDaVinciEndoscopeFocus.h> // should have a proxy

#include <sawIntuitiveResearchKit/mtsDaVinciEndoscopeFocusQtWidget.h>
#include <sawIntuitiveResearchKit/system_Qt_widget.h>
#include <sawIntuitiveResearchKit/console_Qt_widget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJQtWidget.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationPSMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationECMQtWidget.h>

#include <QTabWidget>

typedef dvrk::system_Qt dvrk_system_Qt;
CMN_IMPLEMENT_SERVICES(dvrk_system_Qt);

dvrk::system_Qt::system_Qt(void)
{
}

void dvrk::system_Qt::configure(dvrk::system * system)
{
    mtsComponentManager * component_manager = mtsComponentManager::GetInstance();

    auto * system_widget = new dvrk::system_Qt_widget("system_widget");
    component_manager->AddComponent(system_widget);
    m_connections.Add(system_widget->GetName(), "Main",
                      system->GetName(), "Main");
    m_tab_widget = system_widget->get_components_tab();

    // IOs
    QTabWidget * ioTabWidget;
    if (system->m_arm_proxies.size() > 1) {
        ioTabWidget = new QTabWidget();
        ioTabWidget->setObjectName(QString("IOs"));
        m_tab_widget->addTab(ioTabWidget, "IOs");
    } else {
        ioTabWidget = m_tab_widget;
    }

    for (const auto & iter : system->m_IO_proxies) {
        const std::string & name = iter.first;
        const std::string factory_name = "io_widget_factory_for_" + name;

        auto * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory(factory_name);
        component_manager->AddComponent(robotWidgetFactory);
        // this connect needs to happen now so the factory can figure out the io interfaces
        component_manager->Connect(factory_name, "RobotConfiguration", name, "Configuration");
        robotWidgetFactory->Configure();

        // create all
        mtsRobotIO1394QtWidgetFactory::WidgetListType::const_iterator iterator;
        for (iterator = robotWidgetFactory->Widgets().begin();
             iterator != robotWidgetFactory->Widgets().end();
             ++iterator) {
            ioTabWidget->addTab(*iterator, ((*iterator)->GetName()).c_str());
        }
        if (robotWidgetFactory->ButtonsWidget()) {
            ioTabWidget->addTab(robotWidgetFactory->ButtonsWidget(), "Buttons");
        }
    }

    // Arm and PID widgets
    QTabWidget * pidTabWidget;
    QTabWidget * armTabWidget;
    if (system->m_arm_proxies.size() > 1) {
        pidTabWidget = new QTabWidget();
        pidTabWidget->setObjectName(QString("PIDs"));
        m_tab_widget->addTab(pidTabWidget, "PIDs");
        armTabWidget = new QTabWidget();
        armTabWidget->setObjectName(QString("Arms"));
        m_tab_widget->addTab(armTabWidget, "Arms");
    } else {
        pidTabWidget = m_tab_widget; // use current tab widget
        armTabWidget = m_tab_widget;
    }

    for (const auto & iter : system->m_arm_proxies) {
        mtsIntuitiveResearchKitArmQtWidget * armGUI;
        mtsIntuitiveResearchKitSUJQtWidget * sujGUI;
        mtsPIDQtWidget * pidGUI;

        const std::string & name = iter.first;
        const dvrk::arm_proxy_configuration & config = *(iter.second->m_config);

        switch (config.type) {
        case dvrk::arm_type::MTM:
        case dvrk::arm_type::MTM_DERIVED:
        case dvrk::arm_type::PSM:
        case dvrk::arm_type::PSM_DERIVED:
        case dvrk::arm_type::ECM:
        case dvrk::arm_type::ECM_DERIVED:
            // PID widget
            size_t numberOfJoints;
            if (config.native_or_derived_PSM()) {
                numberOfJoints = 7;
            } else if (config.native_or_derived_MTM()) {
                numberOfJoints = 7;
            } else if (config.native_or_derived_ECM()) {
                numberOfJoints = 4;
            } else {
                numberOfJoints = 0; // can't happen but prevents compiler warning
            }
            pidGUI = new mtsPIDQtWidget(name + "_PID_GUI", numberOfJoints);
            pidGUI->Configure();
            component_manager->AddComponent(pidGUI);
            m_connections.Add(pidGUI->GetName(), "Controller",
                            iter.second->m_PID_component_name, "Controller");
            pidTabWidget->addTab(pidGUI, (name + " PID").c_str());

            // Arm widget
            if (config.native_or_derived_PSM()) {
                armGUI = new mtsIntuitiveResearchKitPSMQtWidget(name + "_GUI");
            } else if (config.native_or_derived_MTM()) {
                armGUI = new mtsIntuitiveResearchKitMTMQtWidget(name + "_GUI");
            } else if (config.native_or_derived_ECM()) {
                armGUI = new mtsIntuitiveResearchKitECMQtWidget(name + "_GUI");
            } else {
                armGUI = new mtsIntuitiveResearchKitArmQtWidget(name + "_GUI");
            }
            armGUI->Configure();
            component_manager->AddComponent(armGUI);
            m_connections.Add(armGUI->GetName(), "Manipulator",
                            iter.second->m_arm_component_name,
                            iter.second->m_arm_interface_name);
            armGUI->setObjectName(name.c_str());
            armTabWidget->addTab(armGUI, name.c_str());

            break;

        case dvrk::arm_type::SUJ_Classic:
        case dvrk::arm_type::SUJ_Si:
        case dvrk::arm_type::SUJ_Fixed:

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM1_SUJ");
            component_manager->AddComponent(sujGUI);
            m_connections.Add(sujGUI->GetName(), "Manipulator", "SUJ", "PSM1");
            armTabWidget->addTab(sujGUI, "PSM1 SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("ECM_SUJ");
            component_manager->AddComponent(sujGUI);
            m_connections.Add(sujGUI->GetName(), "Manipulator", "SUJ", "ECM");
            armTabWidget->addTab(sujGUI, "ECM SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM2_SUJ");
            component_manager->AddComponent(sujGUI);
            m_connections.Add(sujGUI->GetName(), "Manipulator", "SUJ", "PSM2");
            armTabWidget->addTab(sujGUI, "PSM2 SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM3_SUJ");
            component_manager->AddComponent(sujGUI);
            m_connections.Add(sujGUI->GetName(), "Manipulator", "SUJ", "PSM3");
            armTabWidget->addTab(sujGUI, "PSM3 SUJ");

            break;

        case dvrk::arm_type::ECM_GENERIC:
        case dvrk::arm_type::MTM_GENERIC:
        case dvrk::arm_type::PSM_GENERIC:
            {
                // Arm widget
                auto * genericComponentGUI = new QWidget();
                auto * genericComponentLayout = new QHBoxLayout();
                genericComponentGUI->setLayout(genericComponentLayout);
                auto * messageGUI = new mtsMessageQtWidgetComponent(name + "_message_GUI");
                component_manager->AddComponent(messageGUI);
                genericComponentLayout->addWidget(messageGUI);
                m_connections.Add(messageGUI->GetName(), "Component",
                                iter.second->m_arm_component_name,
                                iter.second->m_arm_interface_name);
                auto * timingGUI = new mtsIntervalStatisticsQtWidgetComponent(name + "_timing_GUI");
                component_manager->AddComponent(timingGUI);
                genericComponentLayout->addWidget(timingGUI);
                m_connections.Add(timingGUI->GetName(), "Component",
                                iter.second->m_arm_component_name,
                                iter.second->m_arm_interface_name);
                armTabWidget->addTab(genericComponentGUI, name.c_str());
            }
            break;

        default:
            CMN_LOG_CLASS_INIT_ERROR << "dvrk::system_Qt: arm "
                                     << name
                                     << ": unable to create appropriate Qt widgets for arm of this type"
                                     << std::endl;
        }
    }

    // add teleop widgets in console tabs
    QTabWidget * teleopTabWidget = m_tab_widget; // use current tab by default
    for (const auto & console : system->m_consoles) {
        const std::string name = console.first;
        // first create console widgets
        auto * console_widget = new console_Qt_widget(name + "_widget", system_widget);
        component_manager->AddComponent(console_widget);
        m_connections.Add(system->GetName(), name,
                          console_widget->GetName(), "Main");
        system_widget->get_consoles_tab()->addTab(console_widget, name.c_str());
        m_connections.Add(console_widget->GetName(), "clutch",
                          system->GetName(), name + "/clutch");
        m_connections.Add(console_widget->GetName(), "camera",
                          system->GetName(), name + "/camera");
        m_connections.Add(console_widget->GetName(), "operator_present",
                          system->GetName(), name + "/operator_present");

        // second create the teleop widgets for the console
        if (system->m_consoles.size() > 1) {
            teleopTabWidget = new QTabWidget();
            teleopTabWidget->setObjectName((name + "_widget").c_str());
            m_tab_widget->addTab(teleopTabWidget, name.c_str());
        }

        for (const auto & teleop : console.second->m_teleop_proxies) {
            const std::string name = teleop.first;
            mtsComponent * teleop_widget_component;
            QWidget * teleop_widget;
            switch(teleop.second->type()) {
            case teleop_proxy::PSM:
                {
                    auto * teleop_PSM_widget = new mtsTeleOperationPSMQtWidget(name + "_widget");
                    teleop_widget_component = teleop_PSM_widget;
                    teleop_widget = teleop_PSM_widget;
                }
                break;
            case teleop_proxy::ECM:
                {
                    auto * teleop_ECM_widget = new mtsTeleOperationECMQtWidget(name + "_widget");
                    teleop_widget_component = teleop_ECM_widget;
                    teleop_widget = teleop_ECM_widget;
                }
                break;
            default:
                CMN_LOG_CLASS_INIT_ERROR << " undefined teleop type" << std::endl;
                exit(EXIT_FAILURE);
                break;
            }
            teleop_widget->setObjectName(name.c_str());
            teleopTabWidget->addTab(teleop_widget, name.c_str());
            teleop_widget_component->Configure();
            component_manager->AddComponent(teleop_widget_component);
            m_connections.Add(teleop_widget_component->GetName(), "TeleOperation", name, "Setting");
        }
    }

    // add endoscope focus widget
    // if (system->mDaVinciEndoscopeFocus) {
    //     const std::string name = system->mDaVinciEndoscopeFocus->GetName();
    //     mtsDaVinciEndoscopeFocusQtWidget * endoscopeGUI = new mtsDaVinciEndoscopeFocusQtWidget(name + "_GUI");
    //     endoscopeGUI->Configure();
    //     component_manager->AddComponent(endoscopeGUI);
    //     m_connections.Add(endoscopeGUI->GetName(), "Endoscope", name, "Control");
    //     m_tab_widget->addTab(endoscopeGUI, "Focus");
    // }

    // system_widget->HasTeleOp(hasTeleOp);

    // show all widgets
    m_tab_widget->show();
}

void dvrk::system_Qt::add_tab(QWidget * widget, const std::string & name)
{
    m_tab_widget->addTab(widget, name.c_str());
}
