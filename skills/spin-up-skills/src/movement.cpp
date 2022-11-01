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

// Breaks as soon as there are two valid intersections on the path
// solution straight up isn't updating.
// I'm updating the postion every cycle but I'm not actually taking it into account in my calcuations LOL
void followPath(std::vector<std::vector<double>>& path, double lookForwardRadius, double translationalRPM, double maxRPM) {

    double firstX = path[0][0];
    double firstY = path[0][1];
    double currentIndex = 0;
    double previousX = 0;
    double previousY = 0;
    double p = 1.0;
    double smallNum = 0.000001;

    // once the currentIndex is = to the last point on the path you exit the loop (because you're done)
    // path.size() - 2 so the robot enters other mode once it reaches 2nd to last index of path
    while (currentIndex < path.size() - 2) {
        previousX = positionX;
        previousY = positionY;
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

                if ((std::min(x1, x2) - smallNum > solutionX1) || (solutionX1 > std::max(x1, x2) + smallNum) || (std::min(y1, y2) - smallNum > solutionY1) || (solutionY1 > std::max(y1, y2) + smallNum)) {
                    // "solution" is not within the rectangle created by the sooner and later points
                    solution1 = false;
                }

                if ((std::min(x1, x2) - smallNum > solutionX2) || (solutionX2 > std::max(x1, x2) + smallNum) || (std::min(y1, y2) - smallNum > solutionY2) || (solutionY2 > std::max(y1, y2) + smallNum)) {
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
        }

        double desiredAngle = atan2(driveTowards[0] - positionX, driveTowards[1] - positionY) * 180 / M_PI;

        if (desiredAngle < 0) desiredAngle += 360;

        // positive angular difference -> turn clockwise
        double angularDifference = desiredAngle - imu.get_heading();

        // optimize
        if (angularDifference > 180) {
            angularDifference -= 360;
        } else if (angularDifference < -180) {
            angularDifference += 360;
        }

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

        left_front_mtr.move_velocity(leftRPM);
        left_back_mtr.move_velocity(leftRPM);
        right_front_mtr.move_velocity(rightRPM);
        right_back_mtr.move_velocity(rightRPM);

        pros::delay(50);
    }

    // victory spin
    left_front_mtr.move_velocity(-maxRPM);
    left_back_mtr.move_velocity(-maxRPM);
    right_front_mtr.move_velocity(maxRPM);
    right_back_mtr.move_velocity(maxRPM);
}