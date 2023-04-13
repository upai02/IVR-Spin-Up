#include "pros/llemu.hpp"
#include "robot.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include "misc/PositionTracker.h"

double transverseWheelRad = 3.25 * 0.0254 / 2;
double radialWheelRad = 3.25 * 0.0254 / 2;
double lastTransverseValue = 0;
double lastRadialValue = 0;
double last_x_tracking_offset = 0;
double last_y_tracking_offset = 0;
double positionX = 0;
double positionY = 0;

double initHeading = 90;
double currentHeading = initHeading;
double scale_factor_heading = 1.0;

double toMeters(double value, double wheelRadius) {
    return ((value / 5120.0) * 2 * M_PI * wheelRadius)*2;
}

void initTracker(double initial_X, double initial_Y, double initial_heading) {
    lastTransverseValue = toMeters(horizontal_track.get_value()/1.0, transverseWheelRad);
    lastRadialValue = toMeters(vertical_track.get_value()/4.0, radialWheelRad);
    positionX = initial_X;
    positionY = initial_Y;
    initHeading = initial_heading;
    currentHeading = initHeading;
    imu.set_heading(initHeading);
    last_x_tracking_offset = RADIAL_TRACKING_WHEEL_OFFSET * cos(initHeading * M_PI / 180.0);
    last_y_tracking_offset = RADIAL_TRACKING_WHEEL_OFFSET * sin(initHeading * M_PI / 180.0);
    // pros::lcd::set_text(2, "Position X: " + std::to_string(positionX));
    // pros::lcd::set_text(3, "Position Y: " + std::to_string(positionY));
}

double headingCorrection (double currentRotation) {
    double correctedHeading = fmod((currentRotation*scale_factor_heading), 360.0) + initHeading;

    if (correctedHeading > 360) {
        correctedHeading = fmod(correctedHeading, 360);
    }

    if (correctedHeading < 0) {
        correctedHeading = 360 + correctedHeading;
    }

    return correctedHeading;
}

void updatePosition() {
    imu.set_rotation(0);
    while (true) {
        double currentTransverseValue = toMeters(horizontal_track.get_value()/1.0, transverseWheelRad);
        double currentRadialValue = toMeters(vertical_track.get_value()/4.0, radialWheelRad);

        currentHeading = headingCorrection(imu.get_rotation());

        // std::cout << "Current Heading: " << currentHeading << std::endl;

        double cosine = cos(currentHeading * M_PI / 180.0);
        double sine = sin(currentHeading* M_PI / 180.0);

        double radialDeltaY = (currentRadialValue - lastRadialValue) * cosine;
        double transverseDeltaY = -(currentTransverseValue - lastTransverseValue) * sine; // note the - sign
        double deltaY = radialDeltaY + transverseDeltaY;

        double radialDeltaX = (currentRadialValue - lastRadialValue) * sine;
        double transverseDeltaX = (currentTransverseValue - lastTransverseValue) * cosine;
        double deltaX = radialDeltaX + transverseDeltaX;

        // pros::lcd::set_text(2, "Delta X: " + std::to_string(deltaX));
        // pros::lcd::set_text(3, "Delta Y: " + std::to_string(deltaY));

        lastRadialValue = currentRadialValue;
        lastTransverseValue = currentTransverseValue;

        // pros::lcd::set_text(2, "Position X: " + std::to_string(positionX));
        // pros::lcd::set_text(3, "Position Y: " + std::to_string(positionY));

        double x_tracking_offset = TRANSVERSE_TRACKING_WHEEL_OFFSET * sine;
        double y_tracking_offset = TRANSVERSE_TRACKING_WHEEL_OFFSET * cosine;

        // when pure rotating (x_tracking_offset - last_x_tracking_offset) should = deltaX

        positionX += isnanf(deltaX) ? 0 : deltaX;
        positionY += isnanf(deltaY) ? 0 : deltaY;
        // positionX += deltaX - (x_tracking_offset - last_x_tracking_offset);
        // positionY += deltaY + (y_tracking_offset - last_y_tracking_offset);

        // last_x_tracking_offset = x_tracking_offset;
        // last_y_tracking_offset = y_tracking_offset;

        pros::lcd::set_text(2, "Position X: " + std::to_string(positionX));
        pros::lcd::set_text(3, "Position Y: " + std::to_string(positionY));
        // pros::lcd::set_text(2, "Position X: " + std::to_string(horizontal_track.get_value()/1.0));
        // pros::lcd::set_text(3, "Position Y: " + std::to_string(vertical_track.get_value()/4.0));
        pros::lcd::set_text(7, "Heading: " + std::to_string(currentHeading));

        // pros::lcd::set_text(4, "Transverse Val: " + std::to_string(currentTransverseValue));
        // pros::lcd::set_text(5, "Radial Val: " + std::to_string(currentRadialValue));
        
        // std::cout << "cur heading: " << currentHeading << std::endl;
        // std::cout << "x: " << positionX << std::endl;
        // std::cout << "y: " << positionY << std::endl;
        // std::cout << "ht_get_value: " << horizontal_track.get_value() << std::endl;
        // std::cout << "vt_get_value: " << vertical_track.get_value() << std::endl;

        pros::delay(15);
    }
}