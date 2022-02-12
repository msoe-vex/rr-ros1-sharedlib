#include "lib-rr/kinematics/TankDriveKinematics.h"

TankDriveKinematics::TankDriveKinematics(EncoderConfig encoder_config, TankWheelLocations wheel_locations, 
    Pose current_pose) : IDriveKinematics(encoder_config, current_pose),
        m_wheel_locations(wheel_locations) {

}

void TankDriveKinematics::updateForwardKinematics(IDriveNode::FourMotorDriveEncoderVals encoder_vals) {
    // TODO
}

IDriveKinematics::FourMotorPercentages TankDriveKinematics::inverseKinematics(
        float x, float y, float theta, float max) {
    // TODO
}

IDriveKinematics::FourMotorPercentages TankDriveKinematics::simpleKinematics(
        float left_x, float left_y, float right_x, float right_y, float max) {
    float left = left_y;
    float right = right_y;

    float max_val = std::max({fabs(left), fabs(right), max});

    FourMotorPercentages motor_percentages {
        left / max_val,
        right / max_val
    };

    return motor_percentages;
}

TankDriveKinematics::~TankDriveKinematics() {

}
