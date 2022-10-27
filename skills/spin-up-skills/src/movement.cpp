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
    int weirdDiffCounter = 0;
    int oddPosY = 0;
    int oddDestinationY = 0;
    int notSolution1Chosen = 0;
    int towardsStartingPoint = 0;
    int cycles = 0;
    int onlySol1Chosen = 0;
    int crazyPosX = 0;
    int crazyPosY = 0;
    int noDiscrimFound = 0;
    int breakCounter = 0;
    int noSolutionsValid = 0;

    // once the currentIndex is = to the last point on the path you exit the loop (because you're done)
    // path.size() - 2 so the robot enters other mode once it reaches 2nd to last index of path
    // can't be last index bc only way to get to "last" index is to find solution on segment after it
    // need RELATIVE position between robot and line. Robot can always be at origin, line can be translated
    // based on the position of the robot

    // curernt issue: in some cycles, seemingly randomly, a discriminant isn't being found.
    // 3/5 of cycles things happen correct, 2/5 of cycles discriminant isn't found.
    // x is always realistic
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
                // two intersections

                bool solution1 = true;
                bool solution2 = true;
                double solutionX1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / (pow(segmentDist, 2));
                double solutionX2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / (pow(segmentDist, 2));
                double solutionY1 = (-D * dx + std::abs(dy) * sqrt(discriminant)) / (pow(segmentDist, 2));
                double solutionY2 = (-D * dx - std::abs(dy) * sqrt(discriminant)) / (pow(segmentDist, 2));

                // test for legitness of solutions (find which one we want to drive towards)
                // double robotNextPointDist = sqrt(pow(positionX - laterX, 2) + pow(positionY - laterY, 2));
                double robotNextPointDist = sqrt(pow(x2, 2) + pow(y2, 2));
                if ((std::min(x1, x2) > solutionX1 || solutionX1 > std::max(x1, x2)) || std::min(y1, y2) > solutionY1 || solutionY1 > std::max(y1, y2)) {
                    // "solution" is not within the rectangle created by the sooner and later points
                    solution1 = false;
                }

                if (std::min(x1, x2) > solutionX2 || solutionX2 > std::max(x1, x2) || std::min(y1, y2) > solutionY2 || solutionY2 > std::max(y1, y2)) {
                    solution2 = false;
                }

                if (solution1 && solution2) {
                    // follow one that's closer to next point
                    if (sqrt(pow(solutionX1 - x2, 2) + pow(solutionY1 - y2, 2)) < sqrt(pow(solutionX2 - x2, 2) + pow(solutionY2 - y2, 2))) {
                        driveTowards[0] = solutionX1 + positionX;
                        driveTowards[1] = solutionY1 + positionY;
                        currentIndex = i;
                        breakCounter++;
                        pros::lcd::set_text(6, "Solution 1 closer");
                        break;
                    } else {
                        driveTowards[0] = solutionX2 + positionX;
                        driveTowards[1] = solutionY2 + positionY;
                        currentIndex = i;
                        pros::lcd::set_text(6, "Solution 2 closer");
                        notSolution1Chosen++;
                        breakCounter++;
                        break;
                    }
                } else if (solution1) {
                    // make sure solution is closer to next point than the robot
                    if (sqrt(pow(solutionX1 - x2, 2) + pow(solutionY1 - y2, 2)) < robotNextPointDist) {
                        // solution is behind the robot
                        driveTowards[0] = solutionX1 + positionX;
                        driveTowards[1] = solutionY1 + positionY;
                        currentIndex = i;
                        onlySol1Chosen++;
                        breakCounter++;
                        pros::lcd::set_text(6, "Only solution 1 valid");
                        break;
                    } else {
                        pros::lcd::set_text(6, "INVALID only sol 1");
                    }
                } else if (solution2) {
                    if (sqrt(pow(solutionX2 - x2, 2) + pow(solutionY2 - y2, 2)) < robotNextPointDist) {
                        driveTowards[0] = solutionX2 + positionX;
                        driveTowards[1] = solutionY2 + positionY;
                        currentIndex = i;
                        pros::lcd::set_text(6, "Only solution 2 valid");
                        notSolution1Chosen++;
                        breakCounter++;
                        break;
                    } else {
                        pros::lcd::set_text(6, "INVALID only sol 2");
                    }
                } else {
                    pros::lcd::set_text(6, "INVALID no valid sols");
                }

                noSolutionsValid++;
                // /*
                // if (sqrt(pow(solutionX1 - path[currentIndex][0], 2) + pow(solutionY1 - path[currentIndex][1], 2)) < robotLastPointDist) {
                //     // solution is behind the robot
                //     solution1 = false;
                // } else 
                // */
                // /*
                // if (sqrt(pow(solutionX2 - path[currentIndex][0], 2) + pow(solutionY2 - path[currentIndex][1], 2)) < robotLastPointDist) {
                //     solution2 = false;
                // } else 
                // */


                // if (std::min(soonerX, laterX) < solutionX2 < std::max(soonerX, laterX) == false || std::min(soonerY, laterY) < solutionY2 < std::max(soonerY, laterY) == false) {
                //     solution2 = false;
                // }

                // // if both are still valid choose the one that's closer to the next point.
                // if (solution1 && solution2) {
                //     if (sqrt(pow(solutionX1 - laterX, 2) + pow(solutionY1 - laterY, 2)) < sqrt(pow(solutionX2 - laterX, 2) + pow(solutionY2 - laterY, 2))) {
                //         solution2 = false;
                //     } else {
                //         solution1 = false;
                //     }
                //     // find which one has shorter dist to next point on path and set other to false.
                // }

                // if (solution1 && !solution2) {
                //     driveTowards[0] = solutionX1;
                //     driveTowards[1] = solutionY1;
                //     currentIndex = i;
                //     break;
                // } else if (solution2 && !solution1) {
                //     driveTowards[0] = solutionX2;
                //     driveTowards[1] = solutionY2;
                //     currentIndex = i;
                //     break;
                // }
            } else {
                // if there are no valid solutions to drive towards by the time you get to the end
                // of the loop you should just go back towards the last point you know you reached
                // (which this effectively does)
                driveTowards[0] = path[currentIndex][0];
                driveTowards[1] = path[currentIndex][1];

                pros::lcd::set_text(6, "No discriminant found in full loop");
                noDiscrimFound++;

                // for simplicity we aren't doing code for the low probability case of the line
                // being perfectly tangent to the edge of the circle, instead we just go to else
            }
        }
        // START OF DRIVE TOWARDS POINT

        // calculate angular difference
        // double desiredAngle = atan2(driveTowards[0] - positionX, driveTowards[1] - positionY) * 180 / M_PI;


        // so somehow the point behind the robot is being chosen and my test for that is wrong
        // or....?
        double desiredAngle = atan2(0, driveTowards[1] - positionY) * 180 / M_PI;

        if (desiredAngle < 0) desiredAngle += 360;

        // positive angular difference -> turn clockwise
        double angularDifference = desiredAngle - imu.get_heading();

        // optimize
        if (angularDifference > 180) {
            angularDifference -= 360;
        } else if (angularDifference < -180) {
            angularDifference += 360;
        }

        double rotationalRPM = angularDifference * p; // * p
        

        rotationalRPM = 0;


        // prioritize turning by maintaining rotationalRPM difference (rotationalRPM * 2)
        double leftRPM;
        double rightRPM;
        if (translationalRPM + std::abs(rotationalRPM) > maxRPM) {
            // Limit translational velocity when left or rightRPM would be > maxRPM by
            // maintaining the difference between left and rightRPM by replacing 
            // translationalRPM with maxRPM - abs(rotationalRPM)

            // this also means that if the robot is not within 
            // ((maxRPM - translationalRPM) / p) degrees of its desired angle the robot
            // will only rotate until it gets within that range.
            leftRPM = maxRPM - std::abs(rotationalRPM) + rotationalRPM;
            rightRPM = maxRPM - std::abs(rotationalRPM) - rotationalRPM;
        } else {
            leftRPM = translationalRPM + rotationalRPM;
            rightRPM = translationalRPM - rotationalRPM;
        }

        // drive the robot
        // left_front_mtr.move_velocity(leftRPM);
        // left_back_mtr.move_velocity(leftRPM);
        // right_front_mtr.move_velocity(rightRPM);
        // right_back_mtr.move_velocity(rightRPM);

        cycles++;

        // pros::lcd::set_text(2, "Des angle " + std::to_string(desiredAngle));
        // pros::lcd::set_text(3, "Current index is " + std::to_string(currentIndex));
        // pros::lcd::set_text(4, "Target X " + std::to_string(driveTowards[0]));
        

        pros::lcd::set_text(3, "Only sol 1 chosen: " + std::to_string(onlySol1Chosen));
        pros::lcd::set_text(7, "Target Y " + std::to_string(driveTowards[1]));
        pros::lcd::set_text(8, "Position Y " + std::to_string(positionY));

        if (driveTowards[1] == path[0][1]) {
            towardsStartingPoint++;
            pros::lcd::set_text(2, "Start as goal count: " + std::to_string(towardsStartingPoint));
        }

        pros::lcd::set_text(1, "Robot cycles (for scale): " + std::to_string(cycles));

        pros::lcd::set_text(4, "NO valid sols: " + std::to_string(noSolutionsValid) + " CI: " + std::to_string(currentIndex));
        pros::lcd::set_text(5, "Broke from loop count: " + std::to_string(breakCounter));

        // position y is > driveTowards[1]
        // difference is constantly getting greater (more negative)
        /*
        if (driveTowards[1] - positionY < 0) {
            oddPosY++;
            pros::lcd::set_text(7, "pos/dest diff " + std::to_string(driveTowards[1] - positionY));
        }
        */

        // pros::lcd::set_text(5, "Not sol 1 chosen: " + std::to_string(notSolution1Chosen));

        pros::delay(50);
    }

    pros::lcd::set_text(4, "Entered final endpoint honing sequence");

    // victory spin
    left_front_mtr.move_velocity(-maxRPM);
    left_back_mtr.move_velocity(-maxRPM);
    right_front_mtr.move_velocity(maxRPM);
    right_back_mtr.move_velocity(maxRPM);

    /*
    The plan:
     - get all the info you need from constructor
     - each cycle find points on path that connect to points on circle
     - determine which of these points are the best to go towards by...
        - seeing which is in front of / behind the robot (behind is bad)
        - Seeing what path segments the points belong to
        - if no intersections then go to last point you were going towards
          (which by default is the start of the path)
     - drive and turn towards the point (doing a bunch of math you've done before)
       and can do again

     - do special stuff once you get close to the end

    */
}