#pragma once

#include "api.h"
#include "pursuit/path_pursuit/IPathPursuit.h"
#include "eigen/Eigen/Dense"
#include "pathing/Path.h"
#include "util/Timer.h"
#include "util/PID.h"
#include "util/Constants.h"
#include "math/Pose.h"
#include "util/Logger.h"

using namespace Eigen;

class HolonomicPathPursuit : IPathPursuit {
private:
    PID m_x_pid;
    PID m_y_pid;
    PID m_theta_pid;

public:
    HolonomicPathPursuit(Path path, Timer timer=Timer());

    virtual void startPursuit();

    virtual TargetVelocity getTargetVelocity(Pose current_pose);

    virtual ~HolonomicPathPursuit();
};