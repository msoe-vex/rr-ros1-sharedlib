#pragma once

#include <math.h>
#include "lib-rr/auton/Auton.h"
#include "lib-rr/nodes/subsystems/IDriveNode.h"
#include "lib-rr/nodes/odometry_nodes/OdometryNode.h"
#include "lib-rr/util/PID.h"

class DriveStraightAction : public AutonAction {
private:
    IDriveNode* m_drive_node;
    OdometryNode* m_odometry_node;
    Timer m_timer;
    Pose m_starting_pose;
    Pose m_current_pose;
    PID m_theta_error_PID;
    double m_distance;
    double m_max_velocity;
    double m_max_accel;
    double m_lastSpeed;
    double m_lastTime;
    double m_feedForward;

public:
    DriveStraightAction(IDriveNode* drive_node, OdometryNode* odometry_node, double distance, double max_velocity, 
        double max_accel);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};