/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-03-13

  (C) Copyright 2020-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

#include <hrsv_widget.h>

#include <QHBoxLayout>
#include <QLabel>

hrsv_widget::hrsv_widget(node_ptr_t nodeHandle):
    mNodeHandle(nodeHandle)
{
    setupUi();
    startTimer(50); // 50 ms
    setupROS();
}

hrsv_widget::~hrsv_widget(void)
{
    // stopTimer();
}

void hrsv_widget::setupUi(void)
{
    QHBoxLayout * layout = new QHBoxLayout();
    this->setLayout(layout);

    mLeftLabel = new QLabel("--------------------------");
    mLeftLabel->setAlignment(Qt::AlignLeft);
    layout->addWidget(mLeftLabel);

    mMainLabel = new QLabel("-----------------------------------------------------------------------");
    mMainLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(mMainLabel);

    mRightLabel = new QLabel("-------------------------");
    mRightLabel->setAlignment(Qt::AlignRight);
    layout->addWidget(mRightLabel);
}

void hrsv_widget::setupROS(void)
{
#if ROS1
    mClutchSubscriber =
        std::make_shared<ros::Subscriber>(mNodeHandle->subscribe("footpedals/clutch", 100,
                                                                 &hrsv_widget::ClutchCallback, this));
    mOperatorPresentSubscriber =
        std::make_shared<ros::Subscriber>(mNodeHandle->subscribe("footpedals/operatorpresent", 100,
                                                                 &hrsv_widget::OperatorPresentCallback, this));
    mCameraStateSubscriber =
        std::make_shared<ros::Subscriber>(mNodeHandle->subscribe("MTML_MTMR_ECM/current_state", 100,
                                                                 &hrsv_widget::CameraStateCallback, this));
    mPSMSelectedSubscriber =
        std::make_shared<ros::Subscriber>(mNodeHandle->subscribe("console/teleop/teleop_psm_selected", 100,
                                                                 &hrsv_widget::PSMSelectedCallback, this));
    mMTMRPSM1Subscriber =
        std::make_shared<ros::Subscriber>(mNodeHandle->subscribe("MTMR_PSM1/current_state", 100,
                                                                 &hrsv_widget::MTMRPSMCallback, this));
    mMTMRPSM2Subscriber =
        std::make_shared<ros::Subscriber>(mNodeHandle->subscribe("MTMR_PSM2/current_state", 100,
                                                                 &hrsv_widget::MTMRPSMCallback, this));
    mMTMRPSM3Subscriber =
        std::make_shared<ros::Subscriber>(mNodeHandle->subscribe("MTMR_PSM3/current_state", 100,
                                                                 &hrsv_widget::MTMRPSMCallback, this));
    mMTMLPSM1Subscriber =
        std::make_shared<ros::Subscriber>(mNodeHandle->subscribe("MTML_PSM1/current_state", 100,
                                                                 &hrsv_widget::MTMLPSMCallback, this));
    mMTMLPSM2Subscriber =
        std::make_shared<ros::Subscriber>(mNodeHandle->subscribe("MTML_PSM2/current_state", 100,
                                                                 &hrsv_widget::MTMLPSMCallback, this));
    mMTMLPSM3Subscriber =
        std::make_shared<ros::Subscriber>(mNodeHandle->subscribe("MTML_PSM3/current_state", 100,
                                                                 &hrsv_widget::MTMLPSMCallback, this));
#elif ROS2
    mClutchSubscriber =
        mNodeHandle->create_subscription<ROS_MSG(sensor_msgs, Joy)>("footpedals/clutch", 100,
                                                                    std::bind(&hrsv_widget::ClutchCallback,
                                                                              this,
                                                                              std::placeholders::_1));
    mOperatorPresentSubscriber =
        mNodeHandle->create_subscription<ROS_MSG(sensor_msgs, Joy)>("footpedals/operatorpresent", 100,
                                                                    std::bind(&hrsv_widget::OperatorPresentCallback,
                                                                              this,
                                                                              std::placeholders::_1));
    mCameraStateSubscriber =
        mNodeHandle->create_subscription<ROS_MSG(std_msgs, String)>("MTML_MTMR_ECM/current_state", 100,
                                                                    std::bind(&hrsv_widget::CameraStateCallback,
                                                                              this,
                                                                              std::placeholders::_1));
    mPSMSelectedSubscriber =
        mNodeHandle->create_subscription<ROS_MSG(diagnostic_msgs, KeyValue)>("console/teleop/teleop_psm_selected", 100,
                                                                             std::bind(&hrsv_widget::PSMSelectedCallback,
                                                                                       this,
                                                                                       std::placeholders::_1));
    mMTMRPSM1Subscriber =
        mNodeHandle->create_subscription<ROS_MSG(std_msgs, String)>("MTMR_PSM1/current_state", 100,
                                                                    std::bind(&hrsv_widget::MTMRPSMCallback,
                                                                              this,
                                                                              std::placeholders::_1));
    mMTMRPSM2Subscriber =
        mNodeHandle->create_subscription<ROS_MSG(std_msgs, String)>("MTMR_PSM2/current_state", 100,
                                                                    std::bind(&hrsv_widget::MTMRPSMCallback,
                                                                              this,
                                                                              std::placeholders::_1));
    mMTMRPSM3Subscriber =
        mNodeHandle->create_subscription<ROS_MSG(std_msgs, String)>("MTMR_PSM3/current_state", 100,
                                                                    std::bind(&hrsv_widget::MTMRPSMCallback,
                                                                              this,
                                                                              std::placeholders::_1));
    mMTMLPSM1Subscriber =
        mNodeHandle->create_subscription<ROS_MSG(std_msgs, String)>("MTML_PSM1/current_state", 100,
                                                                    std::bind(&hrsv_widget::MTMLPSMCallback,
                                                                              this,
                                                                              std::placeholders::_1));
    mMTMLPSM2Subscriber =
        mNodeHandle->create_subscription<ROS_MSG(std_msgs, String)>("MTML_PSM2/current_state", 100,
                                                                    std::bind(&hrsv_widget::MTMLPSMCallback,
                                                                              this,
                                                                              std::placeholders::_1));
    mMTMLPSM3Subscriber =
        mNodeHandle->create_subscription<ROS_MSG(std_msgs, String)>("MTML_PSM3/current_state", 100,
                                                                    std::bind(&hrsv_widget::MTMLPSMCallback,
                                                                              this,
                                                                              std::placeholders::_1));
#endif
}

void hrsv_widget::timerEvent(QTimerEvent *)
{
#if ROS1
    ros::spinOnce();
#elif ROS2
    rclcpp::spin_some(mNodeHandle);
#endif
}

void hrsv_widget::ClutchCallback(const ROS_MSG(sensor_msgs, Joy) & message)
{
    if (message.buttons.size() != 1) {
        return;
    }
    mClutch = (message.buttons[0] == 1);
    UpdateLabels();
}

void hrsv_widget::OperatorPresentCallback(const ROS_MSG(sensor_msgs, Joy) & message)
{
    if (message.buttons.size() != 1) {
        return;
    }
    mOperatorPresent = (message.buttons[0] == 1);
    UpdateLabels();
}

void hrsv_widget::CameraStateCallback(const ROS_MSG(std_msgs, String) & message)
{
    mCameraState = message.data;
    UpdateLabels();
}

void hrsv_widget::PSMSelectedCallback(const ROS_MSG(diagnostic_msgs, KeyValue) & message)
{
    if (message.key == "MTMR") {
        mRight.PSM = message.value;
    } else if (message.key == "MTML") {
        mLeft.PSM = message.value;
    }
    UpdateLabels();
}

void hrsv_widget::MTMRPSMCallback(const ROS_MSG(std_msgs, String) & message)
{
    mRight.Status = message.data;
    UpdateLabels();
}

void hrsv_widget::MTMLPSMCallback(const ROS_MSG(std_msgs, String) & message)
{
    mLeft.Status = message.data;
    UpdateLabels();
}

void hrsv_widget::UpdateLabels(void)
{
    std::string message;
    message = mLeft.PSM + ": " + mLeft.Status;
    mLeftLabel->setText(QString(message.c_str()));

    message = mRight.PSM + ": " + mRight.Status;
    mRightLabel->setText(QString(message.c_str()));

    message = "";
    if (mOperatorPresent) {
        message.append("Operator");
    } else {
        message.append("No operator");
    }
    if (mClutch) {
        message.append(":Clutched");
    }
    if (mCameraState == "ENABLED") {
        message.append(":Camera");
    }
    mMainLabel->setText(message.c_str());
}
