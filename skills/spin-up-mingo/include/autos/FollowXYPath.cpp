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
        prevXYPos[0] = iteratorNext->second[0];
        prevXYPos[1] = iteratorNext->second[1];
        iteratorNext++;
        desiredCompletionTime = iteratorNext->first;

        for (int i = 0; i < iteratorNext->second.size(); i++) {
            desXYPos[i] = iteratorNext->second[i];
        }

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
            // reversed flips direction AND switches which motor receives which input
        } else if (deltaAngle < -90) {
            deltaAngle += 180;
            reversed = true;
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
            
            updateDesiredPosition();

            relativeTime = pros::c::millis();

            readyForLoop = true;
            count++;
        }
    }

    std::array<double, 2> executePathLoop() {    
        double currentTimeDouble = (pros::c::millis() - relativeTime) / 1000;
        // convert to int and drop decimal
        int currentTimeSec = (int) currentTimeDouble;

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
            double power = deltaAngle * p;

            /*
            psssshhhhh imagine flipping the speed value every cycle
            if (reversed) {
                speed *= -1;
            }
            */

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

        // Path fully followed, stop moving
        return {0, 0};
    }

};