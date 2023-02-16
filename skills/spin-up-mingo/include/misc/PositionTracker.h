#include "robot.h"

extern double positionX;
extern double positionY;

void PositionTracker(double transverseWheelRadMeters, double radialWheelRadMeters);
void initTracker();
void updatePosition(double imu_heading);