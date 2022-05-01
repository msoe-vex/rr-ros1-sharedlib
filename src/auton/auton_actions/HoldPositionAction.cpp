#include "lib-rr/auton/auton_actions/HoldPositionAction.h"

HoldPositionAction::HoldPositionAction(IDriveNode* drive_node, OdometryNode* odometry_node, float time) :
        m_driveNode(drive_node), 
        m_odometryNode(odometry_node),
        m_timoutTime(time),
        m_holdingPID(0.35, 0, 0)
         {

}

void HoldPositionAction::ActionInit() {
    m_timer.Start();
    m_startingPose = m_odometryNode->getCurrentPose();
}

AutonAction::actionStatus HoldPositionAction::Action() {
    m_currentPose = m_odometryNode->getCurrentPose();
    float x_error = m_currentPose.position.x() - m_startingPose.position.x();
    m_motorOutput = MAX_MOTOR_VOLTAGE * m_holdingPID.calculate(x_error);
    m_driveNode->setDriveVoltage(m_motorOutput, 0, 0);

    if(m_timer.Get() > m_timoutTime){
        return CONTINUE;
    } else {
        return END;
    }
    
}

void HoldPositionAction::ActionEnd() {

}