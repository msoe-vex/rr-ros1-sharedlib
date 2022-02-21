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
    float currentX = current_pose.position().x();
    float currentY = current_pose.position().y();

    //Extracting the Poses of the path points before and after the current position?
    Pose Pt1Pose = Pt1.getPose();
    Pose Pt2Pose = Pt2.getPose();

    //starting range-based for loop to sift through path vector
    int startingIndex = 0;
    bool intersectFound = false;
    
    vector<PathPoint> PathPoints = path.getPathPoints;
    vector<float> goalPt;
    //jank syntax for getting the size of the path
    //this is all very jank, need to fix it if the idea works
    //this for loop is wrong, needs to run from StartingIndex to the length of PathPoints, 
    //I think erasing the earliest element in PathPoints when needed should if it
    for(int i = 0 ; PathPoints.size()) {
        //getting offset values for line-circle algortithm
        float x1 = (PathPoints[i].getPose().x()) - currentX;
        float y1 = (PathPoints[i].getPose().y()) - currentY;
        float x2 = (PathPoints[i+1].getPose().x()) - currentX;
        float y2 = (PathPoints[i+1].getPose().y()) - currentY;
        float dx = x2 - x1;
        float dy = y2 - y1;
        float D = (x1*y2) - (x2*y1);
        //see if theres a better alterative to pow
        float dr = sqrt(pow(dx, 2.)+ pow(dy, 2.));
        float discriminant = pow(m_lookAheadDist, 2.) * pow(dr, 2.) - pow(D, 2.);

        //check how c++ does comparision operators
        if (discriminant >= 0) {
            //how to call functions from the math file? Also once I know how to do that i can clean this code up
            //they use np.sqrt instead of a basic sqrt, ask about that
            float sol_x1 = (D* dy + sgn(dy) * dx * sqrt(discriminant))/ pow(dr, 2);
            float sol_x2 = (D* dy - sgn(dy) * dx * sqrt(discriminant))/ pow(dr, 2);
            float sol_y1 = (-D* dx + fabs(dy) * dx * sqrt(discriminant))/ pow(dr, 2);
            float sol_y2 = (-D* dx - fabs(dy) * dx * sqrt(discriminant))/ pow(dr, 2);

            //the x and y coordinates of the two solutions, should probably be in a vector2d data structure
            //is this syntax valid?
            vector<float> sol1((sol_x1 + currentX), (sol_y1 + currentY));
            vector<float> sol2((sol_x2 + currentX), (sol_y2 + currentY));

            //calculating the minimum and maximum acceptable x and y values for the solutions
            //may need to include algorithm header file to use min and max functions
            float minX =  min(PathPoints[i].getPose().x(), PathPoints[i+1].getPose().x());
            float minY =  min(PathPoints[i].getPose().y(), PathPoints[i+1].getPose().y());
            float maxX =  max(PathPoints[i].getPose().x(), PathPoints[i+1].getPose().x());
            float maxY =  max(PathPoints[i].getPose().y(), PathPoints[i+1].getPose().y());

            //the logic tree behind choosing the approtiate goal point, either a solution point or not update
            //some parts may be copy pasted from urbana champaigns python code, look there for issues
            //there might be syntax issues with the if statement
            if (((minX <= sol1[0] <= maxX) && (minY <= sol1[1] <= maxY))) || (((minX <= sol2[0] <= maxX) && (minY <= sol2[1] <= maxY))){
                //intersectFound should be scoped right?
                intersectFound = true;

                if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) and ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)){
                    //need to get this function working first
                    if(pt_to_pt_distance(sol_pt1, path[i+1]) < pt_to_pt_distance(sol_pt2, path[i+1])) {
                        goalPt = sol1;
                    } else {
                        goalPt = sol2;
                    }

                }  else {
                    if(minX <= sol1[0] <= maxX) and (minY <= sol1[1] <= maxY){
                        goalPt = sol1;
                    }else {
                        goalPt = sol2;
                    }
                    
                }   
            }

        }
    }

}

float TankPathPursuit::getTurnVelocity(vector<float> goalPoint){

}

//deconstructor
TankPathPursuit::~TankPathPursuit() {

}