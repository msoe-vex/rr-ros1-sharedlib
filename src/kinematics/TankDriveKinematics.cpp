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

TankDriveKinematics::~TankDriveKinematics() {

}
