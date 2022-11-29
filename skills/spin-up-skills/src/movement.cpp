#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include "misc/PositionTracker.h"

int sgn(double number) {
    if (number >= 0) {
        return 1;
    }

    return -1;
}

/**
Optimizes angle, only works for [-360, 360] input, outputs
[-180, 180]
*/
double optimizeAngle(double angle) {
    if (angle > 180) {
        angle -= 360;
    } else if (angle < -180) {
        angle += 360;
    }

    return angle;
}

/**
Angle in degrees for [0, 360], you can also just pass in a point for dx and dy and it
will be compared relative to the origin.
*/
double lineAndAngleAngularDiff(double dx, double dy, double angleDegrees) {
    // dx, dy and not dy, dx because when the robot points towards positive y it's facing 0/360
    double segmentAngle = atan2(dx, dy) * 180 / M_PI;

    // change domain to [0, 360]
    if (segmentAngle < 0) segmentAngle += 360;

    double angularDiff = segmentAngle - angleDegrees;

    return optimizeAngle(angularDiff);
}

void moveMotors(double leftRPM, double rightRPM) {
    left_front_mtr.move_velocity(leftRPM);
    left_back_mtr.move_velocity(leftRPM);
    right_front_mtr.move_velocity(rightRPM);
    right_back_mtr.move_velocity(rightRPM);
}

// Starts spinning in a circle (a circle with a radius) at the end even in straight line case.

// current issue: breaks when last path seg length is exactly 1, also probably need to tune ALIGNMENT_HELPER_MULTIPLIER and look ahead dist
// test what happens if you change value of ALIGN_HELPER_DIST_AWAY from 1 to something different.
// doesn't seem to be affected by lookAheadDist
// has to be the dy for the last two indexes
// final angle deg has to be = to 0

// issue is probably from edge case where added second to last point is directly on top of original 2nd to last point

// additional idea: if added second to last point is further from final point than original last point then resort to spin on spot
// on the basis that there isn't enough space to execute a path coordinate change to dictate final angle
void followPath(std::vector<std::vector<double>>& path, double lookForwardRadius, double translationalRPM, double maxRPM, double finalAngleDeg) {

    double firstX = path[0][0];
    double firstY = path[0][1];
    double currentIndex = 0;
    double p = 1.5;
    double SMALL_NUM = 0.000001;
    // we want to be able to calculate the alignment helper... maybe... perhaps based on velocity...?
    double ALIGNMENT_HELPER_MULTIPLIER = 1.5;
    double ALIGN_HELPER_DIST_AWAY = ALIGNMENT_HELPER_MULTIPLIER * lookForwardRadius;
    double FINAL_LOCATION_TOLERANCE = 0.2; // meters
    double FINAL_ANGLE_TOLERANCE_DEG = 3.0; 
    bool readyForSpin = false;

    double lastSegDX = path[path.size() - 1][0] - path[path.size() - 2][0];
    double lastSegDY = path[path.size() - 1][1] - path[path.size() - 2][1];
    double lastAngleDiff = lineAndAngleAngularDiff(lastSegDX, lastSegDY, finalAngleDeg);

    // if the dist between the original second to last point & the original final point is less than the 
    // dist between new second to last and original final (which is ALIGN_HELPER_DIST_AWAY) then going to pathSecondToLast
    // then the orignial final is going to look pretty silly relative to spinning on spot/have too tight turns,  
    // (> 90 deg), so we should spin on spot instead
    bool spinOnSpot = (std::abs(lastAngleDiff) > 90) || (sqrt(pow(lastSegDX, 2) + pow(lastSegDY, 2)) < ALIGN_HELPER_DIST_AWAY);
    std::vector<double> ORIGINAL_PATH_FINAL = path[path.size() - 1];


    if (!spinOnSpot) {
        pros::lcd::set_text(1, "No SpinOnSpot");
        // calculate and add new 2nd to last point here
        // need slope created by finalAngle extended outwards
        // tan(theta) give slope but in reality if you want y and x offset
        // you can just use sin(theta) for dy and cos(theta) for dx and get
        // new point from that
        double finalAngleRad = finalAngleDeg * M_PI / 180.0;
        double dx = std::sin(finalAngleRad) * ALIGN_HELPER_DIST_AWAY;
        double dy = std::cos(finalAngleRad) * ALIGN_HELPER_DIST_AWAY;
        // dx and dy are in opposite directions of what we need
        // pros::lcd::set_text(5, "dx: " + std::to_string(-dx));
        std::vector<double> pathSecondToLast = {ORIGINAL_PATH_FINAL[0] - dx, ORIGINAL_PATH_FINAL[1] - dy};
        // We want to configure the dist between pathSecondToLast and ORIGINAL_PATH_FINAL to be as small as is
        // viably possible with the robot's turning capabilities.

        // if the potential new second to last point is at the same location as the current second to last don't add it
        if (pathSecondToLast[0] != path[path.size() - 2][0] || pathSecondToLast[1] != path[path.size() - 2][1]) {
            // delete last element of path (but it's stored above)
            path.pop_back();
            // add the new second to last and final back
            path.push_back(pathSecondToLast);
            path.push_back(ORIGINAL_PATH_FINAL);
        } else {
            pros::lcd::set_text(5, "HI");
        }

        // extra final point is pathFinal[0] + (dx / ALIGNMENT_HELPER_MULTIPLIER) bc dx is the direction we want
        // * ALIGNMENT_HELPER_MULTIPLIER * lookForwardRadius so that just undoes the * ALIGNMENT_HELPER_MULTIPLIER
        // Purpose of extra point is so when the robot goes over the true last point by the
        // slightest amount the point it's trying to go towards (on look ahead circle) will go "off" the 
        // end of the path we can add code so that when this happens when the robot is at the end of the 
        // path it stops and this program ends.
        std::vector<double> extraPoint = {ORIGINAL_PATH_FINAL[0] + (dx / ALIGNMENT_HELPER_MULTIPLIER), ORIGINAL_PATH_FINAL[1] + (dy / ALIGNMENT_HELPER_MULTIPLIER)};
        path.push_back(extraPoint);
        pros::lcd::set_text(2, "3rd to last: " + std::to_string(std::round(pathSecondToLast[0] * 1000) / 1000.0) + ", " + std::to_string(pathSecondToLast[1]));
        pros::lcd::set_text(3, "Last: " + std::to_string(std::round(extraPoint[0] * 1000) / 1000.0) + ", " + std::to_string(extraPoint[1]));
    } else {
        pros::lcd::set_text(1, "Yes SpinOnSpot");
        // extend last segment by lookAheadRadius so same end path functionality can be used
        // regardless of if we're spinning on spot
        double finalSegAngle = atan2(lastSegDX, lastSegDY) * 180 / M_PI;
        if (finalSegAngle < 0) finalSegAngle += 360;
        double dx = sin(finalSegAngle * M_PI / 180.0) * lookForwardRadius;
        double dy = cos(finalSegAngle * M_PI / 180.0) * lookForwardRadius;
        std::vector<double> pathNewLast = {ORIGINAL_PATH_FINAL[0] + dx, ORIGINAL_PATH_FINAL[1] + dy};
        path.push_back(pathNewLast);
    }

    // once the currentIndex is = to the last point on the path you exit the loop (because you're done)
    // path.size() - 2 so the robot enters other mode once it reaches 2nd to last index of path

    // currentIndex < path.size() - 2
    // currentIndex < path.size() - 1
    while (currentIndex < path.size() - 1) {
        updatePosition(imu.get_heading());

        std::vector<double> driveTowards = {path[currentIndex][0], path[currentIndex][1]};

        for (int i = currentIndex; i < path.size() - 1; i++) {
            double x1 = path[i][0] - positionX;
            double x2 = path[i + 1][0] - positionX;
            double y1 = path[i][1] - positionY;
            double y2 = path[i + 1][1] - positionY;

            double dx = x2 - x1;
            double dy = y2 - y1;
            double segmentDist = std::sqrt(pow(dx, 2) + pow(dy, 2));
            double D = (x1 * y2) - (x2 * y1);

            // discriminant gives # of intersections between circle and line
            double discriminant = (pow(lookForwardRadius, 2) * pow(segmentDist, 2)) - pow(D, 2);

            if (discriminant >= 0) {
                // At least one intersection has been found
                bool solution1 = true;
                bool solution2 = true;
                double solutionX1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / (pow(segmentDist, 2));
                double solutionX2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / (pow(segmentDist, 2));
                double solutionY1 = (-D * dx + std::abs(dy) * sqrt(discriminant)) / (pow(segmentDist, 2));
                double solutionY2 = (-D * dx - std::abs(dy) * sqrt(discriminant)) / (pow(segmentDist, 2));

                // test for legitness of solutions (find which one we want to drive towards)
                // double robotNextPointDist = sqrt(pow(positionX - laterX, 2) + pow(positionY - laterY, 2));
                double robotNextPointDist = sqrt(pow(x2, 2) + pow(y2, 2));

                if ((std::min(x1, x2) - SMALL_NUM > solutionX1) || (solutionX1 > std::max(x1, x2) + SMALL_NUM) || (std::min(y1, y2) - SMALL_NUM > solutionY1) || (solutionY1 > std::max(y1, y2) + SMALL_NUM)) {
                    // "solution" is not within the rectangle created by the sooner and later points
                    solution1 = false;
                }

                if ((std::min(x1, x2) - SMALL_NUM > solutionX2) || (solutionX2 > std::max(x1, x2) + SMALL_NUM) || (std::min(y1, y2) - SMALL_NUM > solutionY2) || (solutionY2 > std::max(y1, y2) + SMALL_NUM)) {
                    solution2 = false;
                }

                if (solution1 && solution2) {
                    // follow one that's closer to next point
                    if (sqrt(pow(solutionX1 - x2, 2) + pow(solutionY1 - y2, 2)) < sqrt(pow(solutionX2 - x2, 2) + pow(solutionY2 - y2, 2))) {
                        driveTowards[0] = solutionX1 + positionX;
                        driveTowards[1] = solutionY1 + positionY;
                        currentIndex = i;
                        break;
                    } else {
                        driveTowards[0] = solutionX2 + positionX;
                        driveTowards[1] = solutionY2 + positionY;
                        currentIndex = i;
                        break;
                    }
                } else if (solution1 && (sqrt(pow(solutionX1 - x2, 2) + pow(solutionY1 - y2, 2)) < robotNextPointDist)) {
                    // make sure solution is closer to next point than the robot
                    // solution is behind the robot if second part of condition is false
                    driveTowards[0] = solutionX1 + positionX;
                    driveTowards[1] = solutionY1 + positionY;
                    currentIndex = i;
                    break;
                } else if (solution2 && (sqrt(pow(solutionX2 - x2, 2) + pow(solutionY2 - y2, 2)) < robotNextPointDist)) {
                    driveTowards[0] = solutionX2 + positionX;
                    driveTowards[1] = solutionY2 + positionY;
                    currentIndex = i;
                    break;
                }
            } else {
                // if there are no valid solutions to drive towards by the time you get to the end
                // of the loop you should just go back towards the last point you know you reached
                // (which this effectively does)
                driveTowards[0] = path[currentIndex][0];
                driveTowards[1] = path[currentIndex][1];
            }

            // pros::lcd::set_text(5, "i: " + std::to_string(i) + ", curI: " + std::to_string(currentIndex));
            // If statement should be true once end of path is reached
            if ((i == currentIndex) && (currentIndex == path.size() - 2) && (sqrt(pow(positionX - ORIGINAL_PATH_FINAL[0], 2) + pow(positionY - ORIGINAL_PATH_FINAL[1], 2)) < FINAL_LOCATION_TOLERANCE)) {
                // look ahead point has (most likely) gone off the edge of the extension on the final point, so the 
                // robot is at the final point and should spin on spot (if applicable) or the program should terminate
                readyForSpin = true;
                break;
            }
        }

        pros::lcd::set_text(4, "Robot angle: " + std::to_string(imu.get_heading()));
        pros::lcd::set_text(5, "Dist from end: " + std::to_string(sqrt(pow(positionX - ORIGINAL_PATH_FINAL[0], 2) + pow(positionY - ORIGINAL_PATH_FINAL[1], 2))));
        pros::lcd::set_text(6, "Y Position: " + std::to_string(positionY));

        if (readyForSpin) break;

        double desiredAngle = atan2(driveTowards[0] - positionX, driveTowards[1] - positionY) * 180 / M_PI;


        if (desiredAngle < 0) desiredAngle += 360;

        // positive angular difference -> turn clockwise
        double angularDifference = desiredAngle - imu.get_heading();

        // optimize
        angularDifference = optimizeAngle(angularDifference);

        double rotationalRPM = angularDifference * p;

        // prioritize turning by maintaining rotationalRPM difference (rotationalRPM * 2)
        double leftRPM;
        double rightRPM;
        if (translationalRPM + std::abs(rotationalRPM) > maxRPM) {
            // Limit translational velocity when left or rightRPM would be > maxRPM by
            // maintaining the difference between left and rightRPM by replacing 
            // translationalRPM with maxRPM - abs(rotationalRPM)

            // this also means that if the robot is not within 
            // (maxRPM / p) degrees of its desired angle the robot
            // will only rotate until it gets within that range.
            leftRPM = maxRPM - std::abs(rotationalRPM) + rotationalRPM;
            rightRPM = maxRPM - std::abs(rotationalRPM) - rotationalRPM;
        } else {
            leftRPM = translationalRPM + rotationalRPM;
            rightRPM = translationalRPM - rotationalRPM;
        }
        pros::lcd::set_text(7, std::to_string(leftRPM));
        moveMotors(leftRPM, rightRPM);
        pros::delay(50);
    }

    // hold up we don't need to check if we're set to spin or not. We should spin
    // regardless of if we decided to earlier or not, and instead base things on
    // the difference between current angle and desired angle
    double degFromFinalAngle = finalAngleDeg - imu.get_heading();
    degFromFinalAngle = optimizeAngle(degFromFinalAngle);
    while (std::abs(degFromFinalAngle) > FINAL_ANGLE_TOLERANCE_DEG) {
        degFromFinalAngle = optimizeAngle(finalAngleDeg - imu.get_heading());
        double rotationalRPM = degFromFinalAngle * p;
        moveMotors(rotationalRPM, -rotationalRPM);
        pros::lcd::set_text(4, "Robot angle: " + std::to_string(imu.get_heading()));
    }

    // finally done!!!
    moveMotors(0.0, 0.0);

    // Delay and reprint final dist to see how inaccurate this is bc no trapezoidal profile
    pros::delay(1500);
    pros::lcd::set_text(5, "Dist from end: " + std::to_string(sqrt(pow(positionX - ORIGINAL_PATH_FINAL[0], 2) + pow(positionY - ORIGINAL_PATH_FINAL[1], 2))));
    pros::lcd::set_text(6, "DONE");
}

/*
Plan for pathEnd:
    - calculate change in angle between current pos and end of path
    - Based on distance and angle change create an ellipse to follow between now and end of path
        - To actually follow ellipse use discriminant method with circle and an ellipse?
        - need to calc location on ellipise that is a distance r from current position
    - Decrease look ahead distance for ellipse as dist to end decreases
        - ellipse section created should always have dist to end decreasing as robot follows path
    - follow the ellipse. End lookahead distance is 0

    Using an ellipse creates a continuous line that changes the robot angle over time
    and results in the correct end angle

    Max delta angle would be 180 degrees bc if > 180 you just go in reverse and flip ellipse
*/
void pathEnd(std::vector<double>& secondToLastPoint, std::vector<double>& lastPoint, double finalAngle) {

}