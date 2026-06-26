/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "rviz/ElevatorStatusPanel.h"

#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

namespace lio_rviz_plugins {

namespace {
constexpr int kTimeoutCheckIntervalMs = 500;
constexpr qint64 kMessageTimeoutMs = 3000;
}

ElevatorStatusPanel::ElevatorStatusPanel(QWidget* parent)
    : rviz::Panel(parent) {
    auto* root_layout = new QVBoxLayout;
    auto* topic_layout = new QHBoxLayout;

    topic_edit_ = new QLineEdit("/LIO/elevator_state");
    subscribe_button_ = new QPushButton("订阅");
    topic_layout->addWidget(new QLabel("状态话题:"));
    topic_layout->addWidget(topic_edit_, 1);
    topic_layout->addWidget(subscribe_button_);

    indicator_ = new QLabel;
    indicator_->setFixedSize(24, 24);

    state_label_ = new QLabel;
    QFont state_font = state_label_->font();
    state_font.setPointSize(18);
    state_font.setBold(true);
    state_label_->setFont(state_font);

    auto* state_layout = new QHBoxLayout;
    state_layout->addStretch();
    state_layout->addWidget(indicator_);
    state_layout->addSpacing(8);
    state_layout->addWidget(state_label_);
    state_layout->addStretch();

    detail_label_ = new QLabel;
    detail_label_->setAlignment(Qt::AlignCenter);
    detail_label_->setWordWrap(true);

    auto* estimate_layout = new QGridLayout;
    displacement_value_ = new QLabel;
    velocity_value_ = new QLabel;
    acceleration_value_ = new QLabel;
    const QList<QLabel*> estimate_values = {
        displacement_value_, velocity_value_, acceleration_value_};
    for (QLabel* value : estimate_values) {
        QFont value_font = value->font();
        value_font.setPointSize(14);
        value_font.setBold(true);
        value->setFont(value_font);
        value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    }
    estimate_layout->addWidget(new QLabel("相对位移"), 0, 0);
    estimate_layout->addWidget(displacement_value_, 0, 1);
    estimate_layout->addWidget(new QLabel("速度"), 1, 0);
    estimate_layout->addWidget(velocity_value_, 1, 1);
    estimate_layout->addWidget(new QLabel("加速度"), 2, 0);
    estimate_layout->addWidget(acceleration_value_, 2, 1);

    root_layout->addLayout(topic_layout);
    root_layout->addSpacing(12);
    root_layout->addLayout(state_layout);
    root_layout->addWidget(detail_label_);
    root_layout->addSpacing(10);
    root_layout->addLayout(estimate_layout);
    root_layout->addStretch();
    setLayout(root_layout);

    connect(subscribe_button_, SIGNAL(clicked()), this, SLOT(subscribeToTopic()));
    connect(topic_edit_, SIGNAL(returnPressed()), this, SLOT(subscribeToTopic()));
    connect(this, SIGNAL(stateMessageReceived(bool,double,double,double)),
            this, SLOT(updateState(bool,double,double,double)),
            Qt::QueuedConnection);

    timeout_timer_ = new QTimer(this);
    connect(timeout_timer_, SIGNAL(timeout()), this, SLOT(checkMessageTimeout()));
    timeout_timer_->start(kTimeoutCheckIntervalMs);

    setDisplayState(DisplayState::Unknown, "等待状态消息");
    setEstimateValues(false);
    subscribeToTopic();
}

void ElevatorStatusPanel::subscribeToTopic() {
    state_sub_.shutdown();
    received_message_ = false;
    setEstimateValues(false);

    const QString topic = topic_edit_->text().trimmed();
    if (topic.isEmpty()) {
        setDisplayState(DisplayState::Unknown, "状态话题不能为空");
        return;
    }

    state_sub_ = nh_.subscribe(topic.toStdString(), 1,
                               &ElevatorStatusPanel::stateCallback, this);
    setDisplayState(DisplayState::Unknown, QString("等待 %1").arg(topic));
    Q_EMIT configChanged();
}

void ElevatorStatusPanel::stateCallback(const lio::ElevatorState::ConstPtr& msg) {
    Q_EMIT stateMessageReceived(msg->in_elevator, msg->displacement,
                                msg->velocity, msg->acceleration);
}

void ElevatorStatusPanel::updateState(bool in_elevator, double displacement,
                                      double velocity, double acceleration) {
    received_message_ = true;
    last_message_.restart();
    setDisplayState(in_elevator ? DisplayState::Elevator : DisplayState::Normal,
                    QString("话题: %1").arg(topic_edit_->text().trimmed()));
    setEstimateValues(in_elevator, displacement, velocity, acceleration);
}

void ElevatorStatusPanel::checkMessageTimeout() {
    if (received_message_ && last_message_.isValid() &&
        last_message_.elapsed() > kMessageTimeoutMs) {
        received_message_ = false;
        setDisplayState(DisplayState::Unknown, "超过 3 秒未收到状态消息");
        setEstimateValues(false);
    }
}

void ElevatorStatusPanel::setEstimateValues(bool available, double displacement,
                                            double velocity, double acceleration) {
    if (!available) {
        displacement_value_->setText("—");
        velocity_value_->setText("—");
        acceleration_value_->setText("—");
        return;
    }
    displacement_value_->setText(QString::number(displacement, 'f', 3) + " m");
    velocity_value_->setText(QString::number(velocity, 'f', 3) + " m/s");
    acceleration_value_->setText(QString::number(acceleration, 'f', 3) + " m/s²");
}

void ElevatorStatusPanel::setDisplayState(DisplayState state, const QString& detail) {
    QString color;
    QString text;
    switch (state) {
        case DisplayState::Normal:
            color = "#2e7d32";
            text = "普通模式";
            break;
        case DisplayState::Elevator:
            color = "#ef6c00";
            text = "电梯模式";
            break;
        case DisplayState::Unknown:
        default:
            color = "#757575";
            text = "状态未知";
            break;
    }

    indicator_->setStyleSheet(
        QString("background-color: %1; border: 2px solid #eeeeee; border-radius: 12px;")
            .arg(color));
    state_label_->setText(text);
    state_label_->setStyleSheet(QString("color: %1;").arg(color));
    detail_label_->setText(detail);
}

void ElevatorStatusPanel::save(rviz::Config config) const {
    rviz::Panel::save(config);
    config.mapSetValue("Topic", topic_edit_->text().trimmed());
}

void ElevatorStatusPanel::load(const rviz::Config& config) {
    rviz::Panel::load(config);
    QString topic;
    if (config.mapGetString("Topic", &topic) && !topic.trimmed().isEmpty()) {
        topic_edit_->setText(topic.trimmed());
        subscribeToTopic();
    }
}

}  // namespace lio_rviz_plugins

PLUGINLIB_EXPORT_CLASS(lio_rviz_plugins::ElevatorStatusPanel, rviz::Panel)
