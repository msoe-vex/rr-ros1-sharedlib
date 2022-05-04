#include "lib-rr/auton/auton_actions/FollowPathAction.h"

IncompletePathAction::IncompletePathAction(IDriveNode* drive_node, OdometryNode* odom_node, IPathPursuit* pursuit, Path path, bool reset_pose) :
        m_drive_node(drive_node),
        m_odom_node(odom_node), 
        m_pursuit(pursuit),
        m_path(path),
        m_reset_pose(reset_pose) {

}

void IncompletePathAction::ActionInit() {
    if (m_reset_pose) {
        m_odom_node->setCurrentPose(m_path.getPathPoints().at(0).getPose());
    }

    m_pursuit->startPursuit();
}

AutonAction::actionStatus IncompletePathAction::Action() {
    // Get current pose
    Vector2d currentPosition = m_odom_node->getCurrentPose().position;
    
    // Get the pose of the final point in the path (where we should be)
    vector<PathPoint> pathPoints = m_path->getPathPoints();
    Vector2d finalPosition = pathPoints[pathPoints.size].getPose().position;

    // Calculate the distance between the two 
    float distance = sqrt(pow(finalPosition.x - currentPosition.x) + pow(finalPosition.y - currentPosition.y));

    if (distance < 4) {
        return END;
    } else {
        m_drive_node->setDriveVelocity(0, 0);
        return CONTINUE;
    }

    return CONTINUE;
}

void IncompletePathAction::ActionEnd() {
    m_drive_node->setDriveVelocity(0, 0, 0);
}