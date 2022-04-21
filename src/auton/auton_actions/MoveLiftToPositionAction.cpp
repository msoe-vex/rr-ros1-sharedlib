#include "lib-rr/auton/auton_actions/MoveLiftToPositionAction.h"

MoveLiftToPositionAction::MoveLiftToPositionAction(ILiftNode* lift_node, int position, int tolerance) : 
        m_lift_node(lift_node), 
        m_position(position),
        m_tolerance(tolerance) {
    
}

void MoveLiftToPositionAction::ActionInit() {

}

AutonAction::actionStatus MoveLiftToPositionAction::Action() {
    m_lift_node->setLiftPosition(m_position, m_tolerance);
    return END;
}

void MoveLiftToPositionAction::ActionEnd() {
    
}