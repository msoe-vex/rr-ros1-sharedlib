#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "util/Constants.h"
#include "nodes/subsystems/ConveyerNode.h"

class ConveyerAction : public AutonAction {
private:
    IConveyerNode* m_conveyer_node;
    Timer m_timer;
    int m_voltage;
    double m_time; // Time in seconds

public:
    ConveyerAction(IConveyerNode* conveyer_node, int voltage=MAX_MOTOR_VOLTAGE, double time=0);

    void ActionInit();

    actionStatus Action();

    void ActionEnd();
};