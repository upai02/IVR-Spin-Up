#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include "misc/PositionTracker.h"
#include "movement.h"

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

double reverseAngle(double angle) {
    if (angle >= 180.0) {
        angle -= 180.0;
    } else {
        angle += 180.0;
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
    left_side.move_velocity(leftRPM);
    right_side.move_velocity(rightRPM);
}

void stopMotors() {
    left_side.move_velocity(0.0);
    right_side.move_velocity(0.0);
}

// This is mainly a random method I've used to test various movement-related code
void SmartStop() {
    stopMotors();
    // double desiredVel = 175.0;
    // moveMotors(desiredVel, desiredVel);
    
    // pros::delay(1000);

    // updatePosition(imu.get_heading());
    // double stopX = positionX;
    // double stopY = positionY;

    // stopMotors();

    pros::delay(1000);

    while (true) {
        updatePosition(imu.get_heading());
        /*
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            desiredVel++;
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            desiredVel = std::max(desiredVel - 1, 0.0);
        }
        moveMotors(desiredVel, desiredVel);        
        pros::lcd::set_text(2, "Left front: " + std::to_string(left_front_mtr.get_actual_velocity()));
        pros::lcd::set_text(3, "Right back: " + std::to_string(right_back_mtr.get_actual_velocity()));
        pros::lcd::set_text(4, "Desired vel: " + std::to_string(desiredVel));
        */


        // pros::lcd::set_text(2, "Stop X: " + std::to_string(stopX));
        // pros::lcd::set_text(3, "Stop Y: " + std::to_string(stopY));
        // pros::lcd::set_text(4, "Current X: " + std::to_string(positionX));
        // pros::lcd::set_text(5, "Current Y: " + std::to_string(positionY));
        // pros::lcd::set_text(6, "Left front brake mode: " + std::to_string(left_front_mtr.get_brake_mode()));
        // pros::lcd::set_text(7, "Back left brake mode: " + std::to_string(left_back_mtr.get_brake_mode()));

        // moveMotors(-30, 30);

        pros::delay(50);
    }
}

double getRotationalRPM(double desiredAngleDeg, bool reversed = false, double p = 1.5) {
    if (reversed) {
        return optimizeAngle(desiredAngleDeg - (reverseAngle(imu.get_heading()))) * p;
    } else {
        return optimizeAngle(desiredAngleDeg - imu.get_heading()) * p;
    }
}

void turnToAngle(double desiredAngleDeg, double toleranceDeg, bool debug, double p) {
    double degFromFinalAngle = desiredAngleDeg - imu.get_heading();
    degFromFinalAngle = optimizeAngle(degFromFinalAngle);
    while (std::abs(degFromFinalAngle) > toleranceDeg) {
        updatePosition(imu.get_heading());
        degFromFinalAngle = optimizeAngle(desiredAngleDeg - imu.get_heading());
        double rotRPM = degFromFinalAngle * p;
        moveMotors(rotRPM, -rotRPM);
        if (debug) pros::lcd::set_text(4, "Robot angle: " + std::to_string(imu.get_heading()));
        pros::delay(50);
    }
}

double calcGoalAngle(std::vector<double> vect) {
    double desiredAngle = atan2(GOAL_X - vect[0], GOAL_Y - vect[1]) * 180 / M_PI;
    if (desiredAngle < 0) desiredAngle += 360;

    return desiredAngle;
}

void followPath(std::vector<std::vector<double>>& path, double finalAngleDeg, bool reversed, bool spinAtEnd, double lookForwardRadius, double final_angle_tolerance_deg, double translationalRPM, double maxRPM, bool printMessages) {
    double firstX = path[0][0];
    double firstY = path[0][1];
    double currentIndex = 0;
    double p = 1.5;
    double SMALL_NUM = 0.000001;
    // A bonus would be able to calculate ALIGNMENT_HELPER_MULTIPLIER based on robot velocity
    double ALIGNMENT_HELPER_MULTIPLIER = 1.5;
    double ALIGN_HELPER_DIST_AWAY = ALIGNMENT_HELPER_MULTIPLIER * lookForwardRadius;
    double FINAL_LOCATION_TOLERANCE = 0.2; // meters
    bool readyForSpin = false;

    double lastSegDX = path[path.size() - 1][0] - path[path.size() - 2][0];
    double lastSegDY = path[path.size() - 1][1] - path[path.size() - 2][1];
    double lastAngleDiff = lineAndAngleAngularDiff(lastSegDX, lastSegDY, finalAngleDeg);

    // If the dist between the original second to last point & the original final point is less than the 
    // dist between new second to last and original final (which is ALIGN_HELPER_DIST_AWAY) then going to the new
    // second to last point doesn't make sense because the robot would ideally drive forwards, then backwards, then forwards again. 
    // So, we check for that and spin on spot at the end if that's the case .
    // There's also a manual override for spinOnSpot that's an optional argument (spinAtEnd) in case the course is too compact.
    bool spinOnSpot = (std::abs(lastAngleDiff) > 90) || (sqrt(pow(lastSegDX, 2) + pow(lastSegDY, 2)) < ALIGN_HELPER_DIST_AWAY) || spinAtEnd;
    std::vector<double> ORIGINAL_PATH_FINAL = path[path.size() - 1];


    if (!spinOnSpot) {
        if (printMessages) pros::lcd::set_text(1, "No SpinOnSpot");
        // Calculate and add new 2nd to last point that will let the robot drive towards the original final point
        // while facing the desired final angle. To do this, a point is added in the opposite direction of the 
        // desired final angle at a distance away equal to ALIGN_HELPER_DIST_AWAY.
        double finalAngleRad = finalAngleDeg * M_PI / 180.0;
        double dx = std::sin(finalAngleRad) * ALIGN_HELPER_DIST_AWAY;
        double dy = std::cos(finalAngleRad) * ALIGN_HELPER_DIST_AWAY;
        std::vector<double> pathSecondToLast = {ORIGINAL_PATH_FINAL[0] - dx, ORIGINAL_PATH_FINAL[1] - dy};

        // In the future we want to configure the dist between pathSecondToLast and ORIGINAL_PATH_FINAL to be as small
        // as possible with the robot's turning capabilities.

        // If the potential new second to last point is at the same location as the current second to last don't add it.
        if (pathSecondToLast[0] != path[path.size() - 2][0] || pathSecondToLast[1] != path[path.size() - 2][1]) {
            // Delete last element of path (but it's stored above)
            path.pop_back();
            // Add the new second to last and final back
            path.push_back(pathSecondToLast);
            path.push_back(ORIGINAL_PATH_FINAL);
        } 

        // Add an extra point at the end of the path that is in the direction of the desired final angle. The distance
        // of this point from the desired stopping point is the lookForwardRadius, as when the look ahead point "falls off"
        // the end of the path created by this extra point the path following section of this program will stop.
        std::vector<double> extraPoint = {ORIGINAL_PATH_FINAL[0] + (dx / ALIGNMENT_HELPER_MULTIPLIER), ORIGINAL_PATH_FINAL[1] + (dy / ALIGNMENT_HELPER_MULTIPLIER)};
        path.push_back(extraPoint);
        if (printMessages) {
            pros::lcd::set_text(2, "3rd to last: " + std::to_string(std::round(pathSecondToLast[0] * 1000) / 1000.0) + ", " + std::to_string(pathSecondToLast[1]));
            pros::lcd::set_text(3, "Last: " + std::to_string(std::round(extraPoint[0] * 1000) / 1000.0) + ", " + std::to_string(extraPoint[1]));
        }
    } else {
        if (printMessages) pros::lcd::set_text(1, "Yes SpinOnSpot");
        // This appends an extra point to the end of the path (similar to how it's done on line 107 with extraPoint)
        // except the point is in the direction of the line between the original final point and original second to last point
        double finalSegAngle = atan2(lastSegDX, lastSegDY) * 180 / M_PI;
        if (finalSegAngle < 0) finalSegAngle += 360;
        double dx = sin(finalSegAngle * M_PI / 180.0) * lookForwardRadius;
        double dy = cos(finalSegAngle * M_PI / 180.0) * lookForwardRadius;
        std::vector<double> pathNewLast = {ORIGINAL_PATH_FINAL[0] + dx, ORIGINAL_PATH_FINAL[1] + dy};
        path.push_back(pathNewLast);
    }

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

            // Discriminant gives # of intersections between circle and line
            double discriminant = (pow(lookForwardRadius, 2) * pow(segmentDist, 2)) - pow(D, 2);

            if (discriminant >= 0) {
                // At least one intersection has been found
                bool solution1 = true;
                bool solution2 = true;
                double solutionX1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / (pow(segmentDist, 2));
                double solutionX2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / (pow(segmentDist, 2));
                double solutionY1 = (-D * dx + std::abs(dy) * sqrt(discriminant)) / (pow(segmentDist, 2));
                double solutionY2 = (-D * dx - std::abs(dy) * sqrt(discriminant)) / (pow(segmentDist, 2));

                // Test for legitness of solutions (find which one we want to drive towards)
                double robotNextPointDist = sqrt(pow(x2, 2) + pow(y2, 2));

                if ((std::min(x1, x2) - SMALL_NUM > solutionX1) || (solutionX1 > std::max(x1, x2) + SMALL_NUM) || (std::min(y1, y2) - SMALL_NUM > solutionY1) || (solutionY1 > std::max(y1, y2) + SMALL_NUM)) {
                    // "solution" is not within the rectangle created by the sooner and later points, so it's invalid
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
                    // Ensure solution1 is closer to next point than the robot. If it isn't, then
                    // the point is an intersection with the back of the look ahead circle and the path, so it's invalid
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
                // If there are no valid solutions to drive towards by the time you get to the end
                // of the loop you should just go back towards the last point you know you reached.
                driveTowards[0] = path[currentIndex][0];
                driveTowards[1] = path[currentIndex][1];
            }

            // If statement should be true once end of path is reached
            if ((i == currentIndex) && (currentIndex == path.size() - 2) && (sqrt(pow(positionX - ORIGINAL_PATH_FINAL[0], 2) + pow(positionY - ORIGINAL_PATH_FINAL[1], 2)) < FINAL_LOCATION_TOLERANCE)) {
                // look ahead point has (most likely) gone off the edge of the extension on the final point, so the 
                // robot is at the final point and should spin on spot (if applicable) or the program should terminate
                readyForSpin = true;
                break;
            }
        }

        if (printMessages) {
            pros::lcd::set_text(4, "Robot angle: " + std::to_string(imu.get_heading()));
            pros::lcd::set_text(5, "Dist from end: " + std::to_string(sqrt(pow(positionX - ORIGINAL_PATH_FINAL[0], 2) + pow(positionY - ORIGINAL_PATH_FINAL[1], 2))));
            pros::lcd::set_text(6, "Y Position: " + std::to_string(positionY));
        }
        
        if (readyForSpin) break;

        double desiredAngle = atan2(driveTowards[0] - positionX, driveTowards[1] - positionY) * 180 / M_PI;

        if (desiredAngle < 0) desiredAngle += 360;

        // Positive angular difference -> turn clockwise
        double rotationalRPM = getRotationalRPM(desiredAngle, reversed);

        // Prioritize turning by maintaining rotationalRPM difference (rotationalRPM * 2)
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
        if (printMessages) pros::lcd::set_text(7, std::to_string(leftRPM));
        if (reversed) {
            moveMotors(-rightRPM, -leftRPM);
        } else {
            moveMotors(leftRPM, rightRPM);
        }
        pros::delay(50);
    }

    moveMotors(0.0, 0.0);
    pros::delay(300); // give the robot time to come to a full stop

    // Turn to face final angle. This runs regardless of spinOnSpot to guarantee we're facing
    // the desired final angle
    turnToAngle(finalAngleDeg, final_angle_tolerance_deg, false, p);
    moveMotors(0.0, 0.0);


    // Delay and reprint final dist from target location to see how inaccurate this is because this isn't using a trapezoidal profile
    // pros::delay(1500); imagine there being a random 1.5 second delay left lying around from testing
    if (printMessages) {
        pros::lcd::set_text(5, "Dist from end: " + std::to_string(sqrt(pow(positionX - ORIGINAL_PATH_FINAL[0], 2) + pow(positionY - ORIGINAL_PATH_FINAL[1], 2))));
        pros::lcd::set_text(6, "DONE");
    }
}

// default x, y coords are the goal/net
void turnToPoint(double pointX, double pointY) {
    double FINAL_ANGLE_TOLERANCE = 2.0;
    double desiredAngle = atan2(pointX - positionX, pointY - positionY) * 180 / M_PI;
    if (desiredAngle < 0) desiredAngle += 360;

    while (std::abs(optimizeAngle(desiredAngle - imu.get_heading())) > FINAL_ANGLE_TOLERANCE) {
        updatePosition(imu.get_heading());
        desiredAngle = atan2(pointX - positionX, pointY - positionY) * 180 / M_PI;
        if (desiredAngle < 0) desiredAngle += 360;
        double rotationalRPM = getRotationalRPM(desiredAngle, false);
        moveMotors(rotationalRPM, -rotationalRPM);

        pros::delay(50);
    }
}