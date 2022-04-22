#include "lib-rr/auton/auton_actions/FollowPathAction.h"

FollowPathAction::FollowPathAction(IDriveNode* drive_node, OdometryNode* odom_node, IPathPursuit* pursuit, Path path, bool reset_pose) :
        m_drive_node(drive_node),
        m_odom_node(odom_node), 
        m_pursuit(pursuit),
        m_path(path),
        m_reset_pose(reset_pose) {

}

void FollowPathAction::ActionInit() {
    if (m_reset_pose) {
        m_odom_node->setCurrentPose(m_path.getPathPoints().at(0).getPose());
    }

    m_pursuit->startPursuit();
}

AutonAction::actionStatus FollowPathAction::Action() {
    Pose currentPose = m_odom_node->getCurrentPose();

    IPursuit::TargetVelocity target_velocity = m_pursuit->getTargetVelocity(currentPose);

    //std::cout << "X: " << currentPose.position.x() << " | Y: " << currentPose.position.y() << " | Angle: " << currentPose.angle.angle() << " | T_X: " << target_velocity.linear_velocity.x() << " | T_Y: " << target_velocity.linear_velocity.y() << " | T_Offset: " << target_velocity.rotational_velocity << std::endl;

    m_drive_node->setDriveVelocity(target_velocity.linear_velocity.x(), target_velocity.linear_velocity.y(), target_velocity.rotational_velocity);

    if (m_timer.Get() == 0 && target_velocity.is_within_end_tolerance) {
        m_timer.Start();
    } else if (m_timer.Get() > 0 && !target_velocity.is_within_end_tolerance) {
        m_timer.Reset();
    } else if (m_timer.Get() > 0.3) {
        return END;
    }

    return CONTINUE;
}

void FollowPathAction::ActionEnd() {
    m_drive_node->setDriveVelocity(0, 0, 0);
}