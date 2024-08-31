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

#ifndef _hrsv_widget_h
#define _hrsv_widget_h

#include <QWidget>

#if ROS1

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <diagnostic_msgs/KeyValue.h>

typedef ros::NodeHandle * node_ptr_t;
#define ROS_MSG(package, message) package::message
#define ROS_SUBSCRIBER(package, message) std::shared_ptr<ros::Subscriber>

#elif ROS2

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

typedef std::shared_ptr<rclcpp::Node> node_ptr_t;
#define ROS_MSG(package, message) package::msg::message
#define ROS_SUBSCRIBER(package, message) typename rclcpp::Subscription<package::msg::message>::SharedPtr

#endif

class QLabel;

class hrsv_widget: public QWidget
{
    Q_OBJECT;

public:
    hrsv_widget(node_ptr_t nodeHandle);
    ~hrsv_widget();

    void setupUi(void);
    void setupROS(void);

private slots:
    void timerEvent(QTimerEvent *);

protected:
    node_ptr_t mNodeHandle;

    QLabel * mLeftLabel;
    QLabel * mRightLabel;
    QLabel * mMainLabel;

    struct {
        std::string PSM;
        std::string Status;
    } mLeft, mRight;

    bool mClutch = false;
    bool mOperatorPresent = false;
    std::string mCameraState;

    ROS_SUBSCRIBER(sensor_msgs, Joy) mClutchSubscriber;
    void ClutchCallback(const ROS_MSG(sensor_msgs, Joy) & message);

    ROS_SUBSCRIBER(sensor_msgs, Joy) mOperatorPresentSubscriber;
    void OperatorPresentCallback(const ROS_MSG(sensor_msgs, Joy) & message);

    ROS_SUBSCRIBER(std_msgs, String) mCameraStateSubscriber;
    void CameraStateCallback(const ROS_MSG(std_msgs, String) & message);

    ROS_SUBSCRIBER(diagnostic_msgs, KeyValue) mPSMSelectedSubscriber;
    void PSMSelectedCallback(const ROS_MSG(diagnostic_msgs, KeyValue) & message);

    // all possible callbacks
    ROS_SUBSCRIBER(std_msgs, String)
        mMTMRPSM1Subscriber,
        mMTMRPSM2Subscriber,
        mMTMRPSM3Subscriber,
        mMTMLPSM1Subscriber,
        mMTMLPSM2Subscriber,
        mMTMLPSM3Subscriber;
    void MTMRPSMCallback(const ROS_MSG(std_msgs, String) & message);
    void MTMLPSMCallback(const ROS_MSG(std_msgs, String) & message);

    void UpdateLabels(void);
};

#endif // _hrsv_widget_h
