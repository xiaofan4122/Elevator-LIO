/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Runtime flag callbacks for elevator-lio.
 */

#include "support/LIONode.h"


void LIONode::elevatorFlagCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!elevator_enable) {
        ELEVATOR_TRIGGER = false;
        LOG_WARN(Elevator, "ignore /LIO/set_elevator_flag because elevator.enable=false");
        return;
    }

    // 修改全局变量
    ELEVATOR_TRIGGER = msg->data;

    // 打印日志方便调试
    if (ELEVATOR_TRIGGER) {
        cout << "ELEVATOR_TRIGGER has been set to TRUE (ON)" << endl;
    } else {
        cout << "ELEVATOR_TRIGGER has been set to FALSE (OFF)" << endl;
    }
}
