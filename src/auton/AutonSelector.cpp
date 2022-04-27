#include "lib-rr/auton/AutonSelector.h"

bool selectAuton(ControllerNode* controllerNode, AutonManagerNode* autonManagerNode) {

    std::vector<Auton*> autons = autonManagerNode->autons;

    std::vector<std::string> autonNames;
    std::for_each(autons.begin(), autons.end(), 
        [&autonNames](Auton* auton) { autonNames.push_back(auton->GetName()); });

    autonManagerNode->selected_auton = autons.at(controllerNode->selectorMenu(autonNames));
    
    return true;
}