/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef LIO_RVIZ_ELEVATOR_STATUS_PANEL_H
#define LIO_RVIZ_ELEVATOR_STATUS_PANEL_H

#include <QElapsedTimer>
#include <QString>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <lio/ElevatorState.h>

class QLabel;
class QLineEdit;
class QPushButton;
class QTimer;

namespace lio_rviz_plugins {

class ElevatorStatusPanel final : public rviz::Panel {
    Q_OBJECT

public:
    explicit ElevatorStatusPanel(QWidget* parent = nullptr);

    void load(const rviz::Config& config) override;
    void save(rviz::Config config) const override;

Q_SIGNALS:
    void stateMessageReceived(bool in_elevator, double displacement,
                              double velocity, double acceleration);

private Q_SLOTS:
    void subscribeToTopic();
    void updateState(bool in_elevator, double displacement,
                     double velocity, double acceleration);
    void checkMessageTimeout();

private:
    enum class DisplayState {
        Unknown,
        Normal,
        Elevator,
    };

    void stateCallback(const lio::ElevatorState::ConstPtr& msg);
    void setDisplayState(DisplayState state, const QString& detail = QString());
    void setEstimateValues(bool available, double displacement = 0.0,
                           double velocity = 0.0, double acceleration = 0.0);

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    QLineEdit* topic_edit_ = nullptr;
    QPushButton* subscribe_button_ = nullptr;
    QLabel* indicator_ = nullptr;
    QLabel* state_label_ = nullptr;
    QLabel* detail_label_ = nullptr;
    QLabel* displacement_value_ = nullptr;
    QLabel* velocity_value_ = nullptr;
    QLabel* acceleration_value_ = nullptr;
    QTimer* timeout_timer_ = nullptr;
    QElapsedTimer last_message_;
    bool received_message_ = false;
};

}  // namespace lio_rviz_plugins

#endif  // LIO_RVIZ_ELEVATOR_STATUS_PANEL_H
