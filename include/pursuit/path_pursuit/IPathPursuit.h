#pragma once

#include "api.h"
#include "pursuit/IPursuit.h"
#include "eigen/Eigen/Dense"
#include "pathing/Path.h"
#include "util/Timer.h"
#include "math/Pose.h"
#include "util/Logger.h"

using namespace Eigen;

class IPathPursuit : IPursuit {
private:
    Path m_path;

public:
    IPathPursuit(Path path, Timer timer=Timer()): IPursuit(timer), m_path(path) {};

    virtual void startPursuit() = 0;

    virtual TargetVelocity getTargetVelocity(Pose current_pose) = 0;

    virtual ~HolonomicPathPursuit() {};
};