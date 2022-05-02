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

    m_startingPose = (m_driveNode->getIntegratedEncoderVals().left_front_encoder_val + m_driveNode->getIntegratedEncoderVals().right_front_encoder_val) / 2.;
}

AutonAction::actionStatus HoldPositionAction::Action() {
    double currentPose = (m_driveNode->getIntegratedEncoderVals().left_front_encoder_val + m_driveNode->getIntegratedEncoderVals().right_front_encoder_val) / 2.;

    float error = currentPose - m_startingPose;
    m_motorOutput = MAX_MOTOR_VOLTAGE * m_holdingPID.calculate(error);
    m_driveNode->setDriveVoltage(0, m_motorOutput, 0);

    if (m_timer.Get() < m_timoutTime){
        return CONTINUE;
    } else {
        return END;
    }
}

void HoldPositionAction::ActionEnd() {

}