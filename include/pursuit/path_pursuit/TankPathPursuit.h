#pragma once

#include "lib-rr/pursuit/path_pursuit/IPathPursuit.h"
#include "lib-rr/pursuit/IPursuit.h"
#include "lib-rr/eigen/Eigen/Dense"
#include "lib-rr/pathing/Path.h"
#include "lib-rr/util/Timer.h"
#include "lib-rr/util/PID.h"
#include "lib-rr/util/Constants.h"
#include "lib-rr/math/Pose.h"
#include "lib-rr/util/Logger.h"
#include<cmath>

using namespace Eigen;

class TankPathPursuit : public IPathPursuit {
private:
    int m_lastFoundIndex;
    float m_lookAheadDist;
    float m_currentHeading;
    bool m_intersectFound;
    float m_pi = atan(1)*4;

public:
    TankPursuit(Path path, Timer timer=Timer());

    virtual void startPursuit();

    virtual float getTurnVelocity(Path path, Pose current_pose);

    virtual ~HolonomicPathPursuit();
};