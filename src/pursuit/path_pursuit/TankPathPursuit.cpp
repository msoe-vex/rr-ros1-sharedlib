#include "lib-rr/pursuit/path_pursuit/TankPathPursuit.h"
#include"math.h"
#include<cmath>
//constructor
TankPathPursuit::TankPathPursuit(Path path, Timer timer) : IPathPursuit(path, timer),
{       
    //whats the best way to use and update this in my getTurnVelocity method?
    m_lastFoundIndex = 0;
        //This value should be in inches
    m_lookAheadDist = 0.7;
        //this value should be in radians
        //probably want to get this from a gryoscope later
    m_currentHeading = 0.;
}

//initialzation of timer instance
void TankPathPursuit::startPursuit() {
    m_timer.Start();
}

//this seems to return a percentage
float TankPathPursuit::getTurnVelocity(Path path, Pose current_pose) {
    Vector2d currentPt = current_pose.position(); //grabs the Vector2d of the current pose passed in
    
    vector<PathPoint> PathPoints = path.getPathPoints();
    intersectFound = false;
    Vector2d goalPt;

    //searches for intersections using the point before and after the robots current position, i and i +1
    //
    for(int i = m_lastFoundIndex; PathPoints.size(); i++) {
        //getting offset values for line-circle algortithm
        Vector2d pt1 = (PathPoints[i].getPose().position()) - currentPt;
        Vector2d pt2 = (PathPoints[i+1].getPose().position()) - currentPt;
        float dx = pt2.x() - pt1.x();
        float dy = pt2.y() - pt1.y();
        float D = (pt1.x()*pt2.y()) - (pt2.x()*pt1.y());
        float dr = sqrt(pow(dx, 2.)+ pow(dy, 2.));
        float discriminant = pow(m_lookAheadDist, 2.) * pow(dr, 2.) - pow(D, 2.);

        //triggered if at least one solution exists, line is tangent or intersecting
        //BIG ISSUE
        //no else for this statement, runs through without defining goalPt, thats an issue
        if (discriminant >= 0) {
            //they use np.sqrt instead of a basic sqrt, ask about that
            float sol_x1 = (D* dy + math::sgn(dy) * dx * sqrt(discriminant))/ pow(dr, 2);
            float sol_x2 = (D* dy - math::sgn(dy) * dx * sqrt(discriminant))/ pow(dr, 2);
            float sol_y1 = (-D* dx + fabs(dy) * dx * sqrt(discriminant))/ pow(dr, 2);
            float sol_y2 = (-D* dx - fabs(dy) * dx * sqrt(discriminant))/ pow(dr, 2);

            Vector2d sol1((sol_x1 + currentX), (sol_y1 + currentY));
            Vector2d sol2((sol_x2 + currentX), (sol_y2 + currentY));

            //calculating the minimum and maximum acceptable x and y values for the solutions
            float minX =  min(pt1.x(), pt2.x());
            float minY =  min(pt1.y(), pt2.y());
            float maxX =  max(pt1.x(), pt2.x());
            float maxY =  max(pt1.y(), pt2.y());

            //this is triggered if one or both solutions are in range
            if (((minX <= sol1.x() <= maxX) && (minY <= sol1.y() <= maxY))) || (((minX <= sol2.x() <= maxX) && (minY <= sol2.y() <= maxY))){
                intersectFound = true;

                //triggered if both solutions are in range and chooses which is a better choice
                if ((minX <= sol1.x() <= maxX) and (minY <= sol1.y() <= maxY)) && ((minX <= sol2.x() <= maxX) and (minY <= sol2.y() <= maxY)){
                    //triggers if sol1 is better
                    if(math::pt_to_pt_distance(sol1, pt2) < math::pt_to_pt_distance(sol2, pt2)) {
                        goalPt = sol1;
                    } else {
                        goalPt = sol2;
                        }
                    }  
                else {
                    if(minX <= sol1.x() <= maxX) && (minY <= sol1.y() <= maxY){
                        goalPt = sol1;
                    }else {
                        goalPt = sol2;
                        }  
                    }
                //triggers if the goalPt is closer the next pt than the current position is
                //this is where i see an issue popping up, this is the only way to break from the for loop so well have to run through the entire vector if this doesnt trigger
                //current_pose wont update in the for loop so thats a big issue, i think
                if (math::pt_to_pt(goalPt, pt2) < math::pt_to_pt(currentPt, pt2)){
                    m_lastFoundIndex = i;
                    break;
                }  else {
                    m_lastFoundIndex = i+1;
                } 
            }
            else {
                intersectFound = false;
                goalPt = PathPoints[m_lastFoundIndex].getPose().position();
            }
        }
    }
    float Kp = 1.2;

    float absTargetAngle = atan2(goalPt.y() - current.y(), goalPt.x() - current.x());
    if (absTargetAngle < 0) {
        absTargetAngle += 360; //make this syntax is right
    }

    float turnError = absTargetAngle - currentHeading;

    if (turnError > 0 || turnError < -180) {
        turnError = -1 * math::sgn(turnError) * (360 - abs(turnError));
    }

    float turnErrorRad = acos((goalPt.dot(currentPt))/((goalPt.norm())*(currentPt.norm())); //might work as an alternative to above code???
    float turnError = turnErrorRad *(180/m_pi);//how to best get pi in here???

    float turnVel = Kp * turnError;

    if (fabs(turnVel) > 100) {
        if (turnVel > 0) {
            turnVel = 100;
        }else {
            turnVel = -100;
        }
    }

    return turnVel;
}

float TankPathPursuit::getTurnVelocity(vector<float> goalPoint){

}

TankPathPursuit::~TankPathPursuit() {

}