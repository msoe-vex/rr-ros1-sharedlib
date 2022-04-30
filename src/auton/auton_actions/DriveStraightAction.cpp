#include "lib-rr/auton/auton_actions/DriveStraightAction.h"

DriveStraightAction::DriveStraightAction(IDriveNode* drive_node, OdometryNode* odometry_node, double distance, double max_velocity, double max_accel) :
        m_drive_node(drive_node), 
        m_odometry_node(odometry_node),
        m_theta_error_PID(0.35, 0, 0),
        m_distance(distance), // in inches
        m_max_velocity(max_velocity), 
        m_max_accel(max_accel), 
        m_lastSpeed(0){

}

void DriveStraightAction::ActionInit() {
    m_timer.Start();
    m_drive_node->resetEncoders();
    m_starting_pose = m_odometry_node->getCurrentPose(); //used to keep track of the starting angle
}

AutonAction::actionStatus DriveStraightAction::Action() {
    double dt = m_timer.Get() - m_lastTime;

    //getting the error in the x direction
    m_current_pose = m_odometry_node->getCurrentPose(); //used to keep track of the current x position
    double theta_error = (m_current_pose.angle * m_starting_pose.angle.inverse()).smallestAngle();
    //calculating the offset in motor velocity due to the x error
    //std::cout << "current calculated theta error: " << theta_error << std::endl;

    double speed = m_max_velocity;

    double test = 0.0;
    std::cout<<"Test: "<<test<<std::endl;
    std::cout<<"Last speed: "<<m_lastSpeed<<std::endl;
    double accel = (m_max_velocity - m_lastSpeed) / dt;

    if (accel < -m_max_accel) {
        std::cout<<"Small accel "<<std::endl;
        std::cout<<"Accel: "<<accel<<" "<<std::endl;
        speed = m_lastSpeed - m_max_accel * dt;
    } else if (accel > m_max_accel) {
        std::cout<<"Big accel "<<std::endl;
        speed = m_lastSpeed + m_max_accel * dt;
    }

    // Subtract the found offset of 3 inches to shorten the path
    double remainingDistance = max(fabs(m_distance) - fabs(((m_drive_node->getIntegratedEncoderVals().left_front_encoder_val / 900.0) * (5./3.) * (M_PI * 3.25))), 0.);
    std::cout << "Remaining Distance: " << remainingDistance << std::endl;
    //std::cout << "Encoder Value: " << m_drive_node->getIntegratedEncoderVals().left_front_encoder_val << std::endl;
    //std::cout << "Current Angle: " << m_current_pose.angle.angle() << " Starting Angle: " << m_starting_pose.angle.angle() <<  " Remaining Dist: " << remainingDistance <<std::endl;


    double maxAllowedSpeed = sqrt(2. * m_max_accel * remainingDistance);
    if (fabs(speed) > maxAllowedSpeed) {
        speed = std::copysign(maxAllowedSpeed, speed);
    }

    speed = max(speed, m_feedForward);
    std::cout << "speed: " << speed << std::endl;

    //calculating motor offset
    double offset = m_theta_error_PID.calculate(theta_error) * speed;

    m_lastSpeed = speed;
    //calculating left and right speeds
    double leftSpeed = speed + offset;

    double rightSpeed = speed - offset;

    //std::cout << "offset: " << offset << " leftSpeed: " << leftSpeed << " rightSpeed: " << rightSpeed << std::endl;

    m_lastTime = m_timer.Get();
    if (remainingDistance < 0.5) {
        return END;
    } else {
        if (m_distance > 0) {
            // m_drive_node->setDriveVelocity(speed, offset);
            m_drive_node->setDriveVelocity(speed, -offset);
        } else {
            m_drive_node->setDriveVelocity(-speed, offset);
        }
        
        return CONTINUE;
    }
}

void DriveStraightAction::ActionEnd() {
    m_drive_node->setDriveVelocity(0, 0);
}