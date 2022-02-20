#include "lib-rr/pursuit/path_pursuit/TankPathPursuit.h"
#include<cmath>
//constructor
TankPathPursuit::TankPathPursuit(Path path, Timer timer) : IPathPursuit(path, timer),
{
        m_lFoundIndex = 0;
        //This value should be in inches
        m_lookAheadDist = 0.7;
        //this value should be in radians
        //probably want to get this from a gryoscope later
        m_currentHeading = 0;
        m_intersectFound = false;
}

//initialzation of timer instance
void TankPathPursuit::startPursuit() {
    m_timer.Start();
}

float TankPathPursuit::getTurnVelocity(Path path, Pose current_pose) {
    //this shuld grab the current x and y position from the pose struct passed into the function
    float currentX = current_pose.position[0];
    float currentY = current_pose.position[1];

    //Extracting the Poses of the path points before and after the current position?
    Pose Pt1Pose = Pt1.getPose();
    Pose Pt2Pose = Pt2.getPose();

    //starting range-based for loop to sift through path vector
    int startingIndex = 0;
    bool intersectFound = false;
    
    //jank syntax for getting the size of the path
    for(int i = 0; i < path.getPathPoints.size(); i++) {
        //where the actual algothrim goes 
        
    }

}

float TankPathPursuit::getTurnVelocity(vector<float> goalPoint){

}

//deconstructor
TankPathPursuit::~TankPathPursuit() {

}