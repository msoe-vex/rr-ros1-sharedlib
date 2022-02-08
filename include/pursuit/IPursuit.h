#pragma once

#include "api.h"
#include "eigen/Eigen/Dense"
#include "util/Timer.h"
#include "math/Pose.h"
#include "util/Logger.h"

using namespace Eigen;

class IPursuit {
private:
    Timer m_timer;

public:
    struct TargetVelocity {
        Vector2d linear_velocity;
        float rotational_velocity;
        bool end_of_path;
    };

    IPursuit(Timer timer=Timer()): m_timer(timer) {};

    virtual void startPursuit() = 0;

    virtual TargetVelocity getTargetVelocity(Pose current_pose) = 0;

    virtual ~HolonomicPathPursuit() {};
};