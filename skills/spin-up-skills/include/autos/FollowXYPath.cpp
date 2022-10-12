#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include <array>
#include <cmath>
#include <map>
#include <string>
#include "robot.h"

class FollowXYPath {
    // each class in c++ needs to have a header file with .h in include, then the actual class itself
    // so I need to add a header file for this class called FollowXYPath.h in include
    // team 453X uses pros on their github, look their for examples :)

    private:
        std::map<double, std::array<double, 2>> theCoords;
        double wheelRadius;
        bool readyForLoop = false;
        std::map<double, std::array<double, 2>>::iterator iteratorNext;
        double previousTime;
        // x, y
        std::array<double, 2> prevXYPos;
        double previousPosition;


        double p = 3;
        bool reversed;
        double relativeTime = 0;
        double desiredCompletionTime;
        double desiredPosition;
        std::array<double, 2> desXYPos;
        double desAngle;
        double deltaPos;
        double deltaAngle;
        double LOWEST_OK_ANGLE_DIFF = 3;
        double desiredRotations;
        double timeDiffMin;
        double speed;
        double count = 0;
        
        bool doneWithPath = false;
    
    void updateDesiredPosition() {
        previousTime = iteratorNext->first;
        // previousPosition = iteratorNext->second;
        prevXYPos[0] = iteratorNext->second[0];
        prevXYPos[1] = iteratorNext->second[1];
        // have to call iteratorNext++ here bc apparently you can't access other variables that are defined in the private variables section :/
        iteratorNext++;
        desiredCompletionTime = iteratorNext->first;
        // desiredPosition = iteratorNext->second;
        // for loop to add two things bc why not
        for (int i = 0; i < iteratorNext->second.size(); i++) {
            desXYPos[i] = iteratorNext->second[i];
        }
        // x, y then 
        desAngle = atan2(desXYPos[0] - prevXYPos[0], desXYPos[1] - prevXYPos[1]) * 180 / 3.1415;
        if (desAngle < 0) {
            desAngle += 360;
        }

        deltaAngle = desAngle - imu.get_heading();

        // optimize
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle < -180) {
            deltaAngle += 360;
        }

        // dist formula
        deltaPos = std::sqrt(std::pow((desXYPos[0] - prevXYPos[0]), 2) + std::pow((desXYPos[1] - prevXYPos[1]), 2));

        desiredRotations = (deltaPos) / (2*3.14159*wheelRadius); // meters to rotations
        timeDiffMin = (desiredCompletionTime - previousTime) / 60;
        speed = desiredRotations / timeDiffMin;
    }

    double getDeltaAngle() {
        deltaAngle = desAngle - imu.get_heading();

        // optimize
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle < -180) {
            deltaAngle += 360;
        }

        if (deltaAngle > 90) {
            deltaAngle -= 180;
            reversed = true;
            // reversed = true
            // reversed should flip direction AND switch which motor receives which input
            // (think about driving a path while turning, then doing that same path with the robot driving backwards)
        } else if (deltaAngle < -90) {
            deltaAngle += 180;
            reversed = true;
            // reversed = true
        }

        return deltaAngle;
    }


    public: 
    
    FollowXYPath(std::map<double, std::array<double, 2>> autoCoords, double wheelRadius) {
        theCoords = autoCoords;
        this->wheelRadius = wheelRadius;
    }

    void initPath() {
        if (readyForLoop == false) {
            iteratorNext = theCoords.begin();
            
            // pros::lcd::set_text(7, "initial time value (0) " + std::to_string(iteratorNext->first));

            updateDesiredPosition();

            relativeTime = pros::c::millis();

            readyForLoop = true;
            count++;

            // pros::lcd::set_text(3, "itNext post-init: " + std::to_string(iteratorNext->first));
        }
    }

    std::array<double, 2> executePathLoop() {    
        double currentTimeDouble = (pros::c::millis() - relativeTime) / 1000;
        // convert to int and drop decimal
        int currentTimeSec = (int) currentTimeDouble;
        // pros::lcd::set_text(4, "itNext: " + std::to_string(iteratorNext->first));
        // pros::lcd::set_text(5, "Current time: " + std::to_string(currentTimeSec));

        if (readyForLoop && !doneWithPath) {
            if (currentTimeSec >= iteratorNext->first) {
                if (iteratorNext == theCoords.end()) {
                    doneWithPath = true;
                } else {
                    updateDesiredPosition();
                    reversed = false;
                    getDeltaAngle();
                }                
            }

            deltaAngle = getDeltaAngle();

            // TO-DO: Make it so the speed considers how long the robot spent turning to desired angle
            // Also TO-DO: make it so the robot is smart enough to drive backwards lol
            double power = deltaAngle * p;

            pros::lcd::set_text(5, "Reversed value is: " + std::to_string(reversed));

            /*
            psssshhhhh imagine flipping the speed value every cycle
            if (reversed) {
                speed *= -1;
            }
            */

            pros::lcd::set_text(6, "Speed val is :    " + std::to_string(speed));

            if (std::abs(deltaAngle) < LOWEST_OK_ANGLE_DIFF) {
                pros::lcd::set_text(7, "Driving");

                // move forward
                if (!reversed) {
                    // normal control
                    return {speed + power, speed - power};
                } else {
                    // flip left/right inputs (in addition to *-1 power from earlier)
                    return {-speed + power, -speed - power};
                }
            } else {
                // turn to angle
                pros::lcd::set_text(7, "Turning");
                return {power, -power};
            }
        }

        return {0, 0};
    }

};