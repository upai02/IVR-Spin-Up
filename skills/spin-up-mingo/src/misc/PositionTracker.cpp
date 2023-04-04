#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include <array>
#include <cmath>
#include <string>
#include "misc/PositionTracker.h"

// 122650 after 6 rots
// 122900, 123000, 122850, 122950
// 20485 ticks per rot

double transverseWheelRad = 1.96 * 0.0254 / 2;
double radialWheelRad = 1.96 * 0.0254 / 2;
int ticks_per_rot = 19600; // 20485;
double lastTransverseValue;
double lastRadialValue;
// double last_x_tracking_offset;
// double last_y_tracking_offset;
double positionX = 0;
double positionY = 0;
bool tracker_initialized = false;

double toMeters(double value, double wheelRadius) {
    return (value / ticks_per_rot) * 2 * M_PI * wheelRadius;
}

// model code on how we could use a mutex if we have issues with either of the positon variables.
// double getPosMutex() {
//     positionX_mutex.take();
//     double temp = positionX;
//     positionX_mutex.give();
//     return temp;
// }

void initTracker(double initial_X, double initial_Y) {
    lastTransverseValue = toMeters(transverseEncoder.get_value(), transverseWheelRad);
    lastRadialValue = toMeters(radialEncoder.get_value(), radialWheelRad);
    // last_x_tracking_offset = RADIAL_TRACKING_WHEEL_OFFSET * cos(imu.get_heading() * M_PI / 180.0);
    // last_y_tracking_offset = RADIAL_TRACKING_WHEEL_OFFSET * sin(imu.get_heading() * M_PI / 180.0);
    positionX = initial_X;
    positionY = initial_Y;
    tracker_initialized = true;
}

bool trackerInitialized() {
    return tracker_initialized;
}

void update_position() {
    while (true) {
        // positionX_mutex.take();
        int32_t transverseEncoderValue = transverseEncoder.get_value();
        int32_t radialEncoderValue = radialEncoder.get_value();
        double currentTransverseValue = toMeters(transverseEncoderValue, transverseWheelRad);
        double currentRadialValue = toMeters(radialEncoderValue, radialWheelRad);
        double heading = imu.get_heading();

        double cosine = cos(heading * M_PI / 180.0);
        double sine = sin(heading * M_PI / 180.0);

        double radialDeltaY = (currentRadialValue - lastRadialValue) * cosine;
        double transverseDeltaY = -(currentTransverseValue - lastTransverseValue) * sine; // note the - sign
        double deltaY = radialDeltaY + transverseDeltaY;

        double radialDeltaX = (currentRadialValue - lastRadialValue) * sine;
        double transverseDeltaX = (currentTransverseValue - lastTransverseValue) * cosine;
        double deltaX = radialDeltaX + transverseDeltaX;

        lastRadialValue = currentRadialValue;
        lastTransverseValue = currentTransverseValue;

        // all wheel offset code is commented out because tracking wheels are currently in the correct positions
        // such that no compensation is needed.
        // double x_tracking_offset = RADIAL_TRACKING_WHEEL_OFFSET * cosine;
        // double y_tracking_offset = RADIAL_TRACKING_WHEEL_OFFSET * sine;

        // when pure rotating (x_tracking_offset - last_x_tracking_offset) should = deltaX


        positionX += deltaX; // - (x_tracking_offset - last_x_tracking_offset);
        positionY += deltaY; // + (y_tracking_offset - last_y_tracking_offset);

        std::cout << pros::c::millis() << ',' << transverseEncoderValue << ',' << positionY << ',' << radialEncoderValue << ',' << positionX << std::endl;


        // last_x_tracking_offset = x_tracking_offset;
        // last_y_tracking_offset = y_tracking_offset;

        // pros::lcd::set_text(4, "intake current: " + std::to_string(intake.get_current_draw()));
        // pros::lcd::set_text(5, "Position X: " + std::to_string(positionX));
        // pros::lcd::set_text(6, "Position Y: " + std::to_string(positionY));
        // pros::lcd::set_text(7, "Heading: " + std::to_string(heading));
        pros::lcd::set_text(4, "Transverse Raw Val: " + std::to_string(transverseEncoder.get_value()));
        pros::lcd::set_text(5, "Transverse Val: " + std::to_string(currentTransverseValue));
        pros::lcd::set_text(6, "Radial Raw Val: " + std::to_string(radialEncoder.get_value()));
        pros::lcd::set_text(7, "Radial Val: " + std::to_string(currentRadialValue));

        // positionX_mutex.give();
        pros::delay(50);
    }
}