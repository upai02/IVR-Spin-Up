#include "robot.h"
#include <array>
#include <cmath>

double transverseWheelRad = 2.75 * 0.0254 / 2;
double radialWheelRad = 2.75 * 0.0254 / 2;
double lastTransverseValue;
double lastRadialValue;
double positionX = 0;
double positionY = 0;

double toMeters(double value, double wheelRadius) {
    return (value / 360.0) * 2 * M_PI * wheelRadius;
}

void initTracker() {
    lastTransverseValue = toMeters(transverseEncoder.get_position() / 100.0, transverseWheelRad);
    lastRadialValue = toMeters(radialEncoder.get_value(), radialWheelRad);
}

void updatePosition(double imu_sensor_heading) {
    double currentTransverseValue = toMeters(transverseEncoder.get_position() / 100.0, transverseWheelRad);
    double currentRadialValue = toMeters(radialEncoder.get_value(), radialWheelRad);

    double cosine = cos(imu_sensor_heading * M_PI / 180.0);
    double sine = sin(imu_sensor_heading * M_PI / 180.0);

    double radialDeltaY = (currentRadialValue - lastRadialValue) * cosine;
    double transverseDeltaY = -(currentTransverseValue - lastTransverseValue) * sine; // note the - sign
    double deltaY = radialDeltaY + transverseDeltaY;

    double radialDeltaX = (currentRadialValue - lastRadialValue) * sine;
    double transverseDeltaX = (currentTransverseValue - lastTransverseValue) * cosine;
    double deltaX = radialDeltaX + transverseDeltaX;

    lastRadialValue = currentRadialValue;
    lastTransverseValue = currentTransverseValue;

    positionX += deltaX;
    positionY += deltaY;
}