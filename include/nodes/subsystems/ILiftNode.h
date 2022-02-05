#pragma once

#include "nodes/NodeManager.h"

class ILiftNode : public Node {
public:
    ILiftNode(NodeManager* node_manager, std::string handle_name);

    virtual void initialize() = 0;

    virtual void setLiftVoltage(int voltage) = 0;

    virtual void setLiftVelocity(int velocity) = 0;

    virtual void setLiftPosition(int position) = 0;

    virtual void teleopPeriodic() {};

    virtual void autonPeriodic() {};

    virtual ~ILiftNode();
};