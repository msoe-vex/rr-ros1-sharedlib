#include "auton/auton_actions/MoveLiftToPositionAction.h"

MoveLiftToPositionAction::MoveLiftToPositionAction(ILiftNode* lift_node, int position, int tolerance) : 
        m_lift_node(lift_node), 
        m_position(position),
        m_tolerance(tolerance) {
    
}

void MoveLiftToPositionAction::ActionInit() {

}

AutonAction::actionStatus MoveLiftToPositionAction::Action() {
    int positionBoundUpper = (m_lift_node->getPosition) + m_tolerance;
    int positionBoundLower = (m_lift_node->getPosition) - m_tolerance;
    if(positionBoundLower < m_position && m_position < positionBoundUpper) {
        return END;
    }
    m_lift_node->setLiftPosition(m_position);
    return CONTINUE;
}

void MoveLiftToPositionAction::ActionEnd() {
    
}