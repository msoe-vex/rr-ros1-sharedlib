#pragma once

#include "nodes/NodeManager.h"

class IConveyorNode : public Node {
public:
    IConveyorNode(NodeManager* node_manager);

    virtual void setConveyorVoltage(int voltage) = 0;

    virtual void setConveyorVelocity(float velocity) = 0;

    virtual void initialize() = 0;

    virtual void teleopPeriodic() {};

    virtual void autonPeriodic() {};

    virtual ~ConveyorNode();
};
