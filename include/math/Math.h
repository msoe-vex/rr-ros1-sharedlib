#pragma once

#include <cmath>
#include <iostream>

#ifndef PI
#define PI 3.141592653589793
#endif

#define kEpsilon .000000001
#define kE .000000001

using namespace std;

inline double toRadians(double degrees) {
    return degrees * (PI / 180.0);
}

inline double toDegrees(double degrees) {
    return degrees * (180.0 / PI);
}

inline double arctan(double x, double y) {
    double result = toDegrees(atan2(y, x));
    if(x >= 0 && y >= 0) {
        return 90 - result;
    } else if(x >= 0 && y < 0) {
        return 90 + -1 * result;
    } else if(x <= 0 && y < 0) {
        return 90 + -1 * result;
    } else if(x < 0 && y >= 0) {
        return 450 - result;
    }
    return -1;
}

inline double pathogram(double x, double y){
    return sqrt(x*x + y*y);
}

inline double clamp(double value, double minimum, double maximum) {
    if(value < minimum) {
        return minimum;
    } else if(value > maximum) {
        return maximum;
    } else {
        return value;
    }
}

inline float pt_to_pt(float pt1x, float pt1y, float pt2x, float pt2y) {
    float distance;
    distance = sqrt(pow((pt1x-pt2x), 2) + pow((pt1y-pt2y), 2));
    return distance;
}

inline float pt_to_pt(Vector2d pt1, Vector pt2) {
    Vector2d difference = pt2 - pt1;
    float distance = sqrt(fabs(pow((difference.x()), 2) + pow((difference.y()), 2)));
    return distance;
}

//inline float pose_to_pose(Pose pose1, Pose pose2){
//    float distance = sqrt(pow((pose1.position.x() - pose2.position.x()), 2) + pow((pose1.position.y() - pose2.position.y()), 2));
//    return distance;
//}

inline int sgn(float number) {
    if (number >= 0) {
        return 1;
    }
    else {
        return -1;
    }
}