#include "pros/rtos.hpp"
#include "robot.h"

extern double positionX;
extern double positionY;
const double RADIAL_TRACKING_WHEEL_OFFSET = 0.0;
inline pros::Mutex positionX_mutex;

void PositionTracker(double transverseWheelRadMeters, double radialWheelRadMeters);
bool trackerInitialized();
void initTracker(double initial_X = 0.0, double initial_Y = 0.0);
void update_position();

double getX();
double getY();

// extern pros::Task position_updater;
