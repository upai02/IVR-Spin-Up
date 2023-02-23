#include "robot.h"

extern double positionX;
extern double positionY;
const double RADIAL_TRACKING_WHEEL_OFFSET = 0.1;

double toMeters(double value, double wheelRadius);
void PositionTracker(double transverseWheelRadMeters, double radialWheelRadMeters);
void initTracker(double initial_X = 0.0, double initial_Y = 0.0);
void updatePosition();