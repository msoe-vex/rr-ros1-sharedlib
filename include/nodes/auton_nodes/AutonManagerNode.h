#pragma once

#include "nodes/NodeManager.h"
#include "auton/auton_routines/TestPathAuton.h"
#include "auton/auton_routines/TestPoseAuton.h"
#include "auton/auton_routines/TestTurnAuton.h"
#include "pathing/PathManager.h"
#include "api.h"

class AutonManagerNode : public Node {
private:
    Auton* m_test_path_auton;
    Auton* m_prog_skills;

public:
    AutonManagerNode(NodeManager* node_manager);

    Auton* selected_auton;

    void initialize();

    void autonPeriodic();
};
