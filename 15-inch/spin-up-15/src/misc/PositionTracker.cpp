#include "pros/llemu.hpp"
#include "robot.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include "misc/PositionTracker.h"

double transverseWheelRad = 2 * 0.0254 / 2;
double radialWheelRad = 2 * 0.0254 / 2;
double lastTransverseValue;
double lastRadialValue;
double last_x_tracking_offset;
double last_y_tracking_offset;
double positionX = 0;
double positionY = 0;
double heading = 0;

double initHeading = 90;
double currentHeading = initHeading;

double toMeters(double value, double wheelRadius) {
    return (value / 360.0) * 2 * M_PI * wheelRadius;
}

void initTracker(double initial_X, double initial_Y) {
    lastTransverseValue = toMeters(horizontal_track.get_value(), transverseWheelRad);
    lastRadialValue = toMeters(vertical_track.get_value(), radialWheelRad);
    last_x_tracking_offset = RADIAL_TRACKING_WHEEL_OFFSET * cos(gps.get_heading() * M_PI / 180.0);
    last_y_tracking_offset = RADIAL_TRACKING_WHEEL_OFFSET * sin(gps.get_heading() * M_PI / 180.0);
    positionX = initial_X;
    positionY = initial_Y;
}

double scale_factor_heading = 1.0;
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
    gps.set_rotation(0);
    while (true) {
        double currentTransverseValue = toMeters(horizontal_track.get_value(), transverseWheelRad);
        double currentRadialValue = toMeters(vertical_track.get_value(), radialWheelRad);

        currentHeading = 360 - gps.get_rotation() + initHeading;

        std::cout << "Current Heading: " << currentHeading << std::endl;

        double cosine = cos(-gps.get_rotation() * M_PI / 180.0);
        double sine = sin(-gps.get_rotation() * M_PI / 180.0);

        double radialDeltaY = (currentRadialValue - lastRadialValue) * cosine;
        double transverseDeltaY = -(currentTransverseValue - lastTransverseValue) * sine; // note the - sign
        double deltaY = radialDeltaY + transverseDeltaY;

        double radialDeltaX = (currentRadialValue - lastRadialValue) * sine;
        double transverseDeltaX = (currentTransverseValue - lastTransverseValue) * cosine;
        double deltaX = radialDeltaX + transverseDeltaX;

        lastRadialValue = currentRadialValue;
        lastTransverseValue = currentTransverseValue;

        double x_tracking_offset = RADIAL_TRACKING_WHEEL_OFFSET * cosine;
        double y_tracking_offset = RADIAL_TRACKING_WHEEL_OFFSET * sine;

        // when pure rotating (x_tracking_offset - last_x_tracking_offset) should = deltaX

        positionX += deltaX;
        positionY -= deltaY;
        // positionX += deltaX - (x_tracking_offset - last_x_tracking_offset);
        // positionY += deltaY + (y_tracking_offset - last_y_tracking_offset);

        // last_x_tracking_offset = x_tracking_offset;
        // last_y_tracking_offset = y_tracking_offset;

        pros::lcd::set_text(3, "Position X: " + std::to_string(positionX));
        pros::lcd::set_text(4, "Position Y: " + std::to_string(positionY));
        pros::lcd::set_text(5, "Heading: " + std::to_string(gps.get_heading()));

        pros::lcd::set_text(6, "Transverse Val: " + std::to_string(currentTransverseValue));
        pros::lcd::set_text(7, "Radial Val: " + std::to_string(currentRadialValue));

        pros::delay(10);
    }
}