#include "auton/auton_actions/ConveyerAction.h"

ConveyerAction::ConveyerAction(IConveyerNode* conveyer_node, int voltage, double time) : 
        m_conveyer_node(conveyer_node), 
        m_voltage(voltage), 
        m_time(time) {
    
}

void ConveyerAction::ActionInit() {
    m_timer.Start();
}

AutonAction::actionStatus ConveyerAction::Action() {
    if (m_time <= 0) {
        m_conveyer_node->setConveyerVoltage(m_voltage);
        return END;
    } else {
        // Run until elapsed time is reached
        if (m_timer.Get() < m_time) {
            m_conveyer_node->setConveyerVoltage(m_voltage);
            return CONTINUE;
        } else {
            m_conveyer_node->setConveyerVoltage(0);
            return END;
        }
    }
}

void conveyerAction::ActionEnd() {
    
}