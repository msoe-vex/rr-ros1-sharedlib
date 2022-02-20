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

using namespace Eigen;

class TankPathPursuit : public IPathPursuit {
private:
    int m_lFoundIndex;
    float m_lookAheadDist;
    float m_currentHeading;
    bool m_intersectFound;

public:
    TankPursuit(Path path, Timer timer=Timer());

    virtual void startPursuit();

    //once I understand vector2d that might be what I want to return
    //virtual vector<float> getGoalPoint(PathPoint Pt1, PathPoint Pt2, Pose current_pose);

    virtual float getTurnVelocity(Path path, Pose current_pose);

    virtual ~HolonomicPathPursuit();
};