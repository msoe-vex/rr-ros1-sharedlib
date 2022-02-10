#pragma once

#include "lib-rr/nodes/NodeManager.h"

class IClawNode : public Node {
    NodeManager* m_node_manager;
    std::string m_handle_name;

public:
    IClawNode(NodeManager* node_manager, std::string handle_name): Node(node_manager, 10), m_node_manager(node_manager), m_handle_name(handle_name) {};

    virtual void initialize() = 0;

    virtual void openClaw() = 0;

    virtual void closeClaw()= 0;

    virtual void teleopPeriodic() {};

    virtual void autonPeriodic() {};

    virtual ~IClawNode();
};