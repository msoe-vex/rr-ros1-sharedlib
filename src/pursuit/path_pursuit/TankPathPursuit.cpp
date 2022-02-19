#include "lib-rr/pursuit/path_pursuit/TankPathPursuit.h"

TankPathPursuit::TankPathPursuit(Path path, Timer timer) : IPathPursuit(path, timer),
        m_x_pid(0.03, 0., 0., 0.),
        m_y_pid(0.03, 0., 0., 0.),
        m_theta_pid(0.4, 0., 0., 0.) {
    
}

void TankPathPursuit::startPursuit() {
    m_timer.Start();
}

IPursuit::goalPoint TankPathPursuit::getGoalPoint(Path path,Pose current_pose) {
    bool intersectionFound = false;

    float currentX = currentPose[0];
    float currentY = currentPose[1];

    float x1 = path[0]; // represents the first pt 1 x coordinate
    float y1 = path[o]; //represents the first pt 1 y coordinate
    float x2 = path[1]; // represents the first pt 2 x coordinate
    float y2 = path[1]; //represents the first pt 2 y coordinate

}

IPursuit::TargetVelocity TankPathPursuit::getTargetVelocity(Pose current_pose) {
    Pose next_pose = m_path.update(m_timer.Get());

    Vector2d linear_error = (current_pose.angle.inverse() * Rotation2Dd(M_PI_2)) * (next_pose.position - current_pose.position);
    float theta_error = (next_pose.angle * current_pose.angle.inverse()).smallestAngle();

    // Determine the feedback of each movement component to get to our new position
    float x_feedback = m_x_pid.calculate(linear_error.x());
    float y_feedback = m_y_pid.calculate(linear_error.y());
    float theta_feedback = m_theta_pid.calculate(theta_error);

    // Return the target velocities, and whether the path is at the end point
    IPursuit::TargetVelocity target_velocity = {
        Vector2d(x_feedback * MAX_VELOCITY, y_feedback * MAX_VELOCITY), 
        theta_feedback * MAX_VELOCITY,
        m_path.isComplete()
    };
    
    return target_velocity;
}

TankPathPursuit::~TankPathPursuit() {

}