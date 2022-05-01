#pragma once

#include <math.h>
#include "lib-rr/auton/Auton.h"
#include "lib-rr/nodes/subsystems/IDriveNode.h"
#include "lib-rr/nodes/odometry_nodes/OdometryNode.h"
#include "lib-rr/util/PID.h"

class HoldPositionAction : public AutonAction {
public:
    HoldPositionAction(IDriveNode* drive_node, OdometryNode* odometry_node, float time);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();

private:
    IDriveNode* m_driveNode;
    OdometryNode* m_odometryNode;
    float m_timoutTime;
    Timer m_timer;
    Pose m_startingPose;
    Pose m_currentPose;
    PID m_holdingPID;
    float m_motorOutput;
};