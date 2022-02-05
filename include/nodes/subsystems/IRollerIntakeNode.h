#pragma once

#include "nodes/NodeManager.h"

class IRollerIntakeNode : public Node {
public:
    IRollerIntakeNode(NodeManager* node_manager, std::string handle_name);

    virtual void initialize() = 0;

    virtual void setIntakeVoltage(int voltage) = 0;

    virtual void setIntakeVelocity(int velocity) = 0;

    virtual void openIntakes(int open) {};

    virtual void teleopPeriodic() {};

    virtual void autonPeriodic() {};

    virtual ~IRollerIntakeNode();
};
