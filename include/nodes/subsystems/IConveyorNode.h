#pragma once

#include "nodes/NodeManager.h"
#include "api.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ADIAnalogInNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"
#include "nodes/actuator_nodes/ADIDigitalOutNode.h"
#include "util/Constants.h"
#include "util/Timer.h"

class IConveyorNode : public Node {
public:
    enum ConveyorState {
        STOPPED, HOLDING, SCORING, REVERSE, DEPLOY, SPLIT, HOLDING_TOP, SCORING_TOP
    };

    IConveyorNode(NodeManager* node_manager);

    virtual void setConveyorVoltage(int voltage) = 0;

    virtual void setConveyorRPMPercent(float percent) = 0;

    virtual void setConveyorState(ConveyorState conveyorState) = 0;

    virtual void initialize() = ;

    virtual void teleopPeriodic() {};

    virtual void autonPeriodic() {};

    virtual ~ConveyorNode();
};
