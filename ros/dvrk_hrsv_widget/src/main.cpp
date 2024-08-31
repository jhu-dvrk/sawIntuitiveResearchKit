/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-03-13

  (C) Copyright 2021-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <hrsv_widget.h>

#include <signal.h>
#include <QApplication>

void signalHandler(int)
{
    QCoreApplication::exit(0);
}

int main(int argc, char *argv[])
{
#if ROS1
    ros::init(argc, argv, "hrsv_widget");
    QApplication application(argc, argv);
    ros::NodeHandle * rosNode = new ros::NodeHandle();
#elif ROS2
    // create ROS node handle
    std::vector<std::string> non_ros_arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto rosNode = std::make_shared<rclcpp::Node>("hrsv_widget");
    QApplication application(argc, argv);
#endif

    hrsv_widget leftWidget(rosNode);
    hrsv_widget rightWidget(rosNode);

    leftWidget.show();
    rightWidget.show();

    signal(SIGINT, signalHandler);
    return application.exec();
}
