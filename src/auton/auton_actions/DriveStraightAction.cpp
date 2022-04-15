#include "lib-rr/auton/auton_actions/DriveStraightAction.h"

DriveStraightAction::DriveStraightAction(IDriveNode* drive_node, OdometryNode* odometry_node, double distance, double max_velocity, 
        double max_accel) :
        m_drive_node(drive_node), 
        m_odometry_node(odometry_node),
        m_distance(distance), // in inches
        m_max_velocity(max_velocity), 
        m_max_accel(max_accel), 
        m_lastSpeed(0), 
        m_feedForward(4.91) {

}

void DriveStraightAction::ActionInit() {
    m_timer.Start();
    m_drive_node->resetEncoders();
    m_starting_pose = m_odometry_node->getCurrentPose(); //used to keep track of the starting x coordinate
}

AutonAction::actionStatus DriveStraightAction::Action() {
    double dt = m_timer.Get() - m_lastTime;

    //getting the error in the x direction
    m_current_pose = m_odometry_node->getCurrentPose(); //used to keep track of the current x position
    double x_error = (m_current_pose.position.x() - m_starting_pose.position.x());
    //calculating the offset in motor velocity due to the x error
    double k = 0.002; //proportional constant for calculating offset
    double offset = k*x_error*m_max_velocity; //offset to be added and subtacted to the left and right sides velocity
    std::cout << "current calculated offset: " << offset << std::endl;

    double speed = m_max_velocity;

    double accel = (m_max_velocity - m_lastSpeed) / dt;

    if (accel < -m_max_accel) {
        speed = m_lastSpeed - m_max_accel * dt;
    } else if (accel > m_max_accel) {
        speed = m_lastSpeed + m_max_accel * dt;
    }

    // Subtract the found offset of 3 inches to shorten the path
    double remainingDistance = max(fabs(m_distance) - fabs(((m_drive_node->getIntegratedEncoderVals().left_front_encoder_val / 900.0) * (M_PI * 4.0625))) - 3, 0.);

    double maxAllowedSpeed = sqrt(2 * m_max_accel * remainingDistance);
    if (fabs(speed) > maxAllowedSpeed) {
        speed = std::copysign(maxAllowedSpeed, speed);
    }

    speed = max(speed, m_feedForward);

    m_lastSpeed = speed;
    //calculating left and right speeds
    double leftSpeed = speed - offset;
    std::cout << "leftSpeed: " << leftSpeed << std::endl;
    double rightSpeed = speed + offset;
    std::cout << "rightSpeed: " << rightSpeed << std::endl;
    m_lastTime = m_timer.Get();
    if (remainingDistance < 0.5) {
        return END;
    } else {
        if (m_distance > 0) {
            m_drive_node->setRightVelocity(rightSpeed);
            m_drive_node->setLeftVelocity(leftSpeed);
        } else {
            m_drive_node->setDriveVelocity(-speed, 0);
        }
        
        return CONTINUE;
    }
}

void DriveStraightAction::ActionEnd() {
    m_drive_node->setDriveVelocity(0, 0);
}