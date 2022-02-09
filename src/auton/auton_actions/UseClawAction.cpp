#include "auton/auton_actions/UseClawAction.h"

UseClawAction::UseClawAction(IClawNode* claw_node, bool position) : 
        m_claw_node(claw_node), 
        m_position(position)  {
    
}

void UseClawAction::ActionInit() {

}

AutonAction::actionStatus UseClawAction::Action() {
    if(m_position) {
        m_claw_node->openClaw();
        return END;
    }
    m_claw_node-> closeClaw();
    return END;
}

void UseClawAction::ActionEnd() {
    
}