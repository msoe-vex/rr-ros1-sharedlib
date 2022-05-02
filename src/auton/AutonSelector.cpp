#include "lib-rr/auton/AutonSelector.h"

bool selectAuton(ControllerNode* controllerNode, AutonManagerNode* autonManagerNode) {

    std::vector<Auton*> autons = autonManagerNode->autons;

    std::vector<std::string> autonNames;
    std::for_each(autons.begin(), autons.end(), 
        [&autonNames](Auton* auton) { autonNames.push_back(auton->GetName()); });

    Auton* newAuton = autons.at(controllerNode->selectorMenu(autonNames));
    
    string autonPath = newAuton->GetAssociatedPath();
    if (autonPath != "") {
        autonManagerNode->setPathsFile(autonPath + ".json");
    }
    
    autonManagerNode->selected_auton = newAuton;

    return newAuton->GetNeedsPath();
}