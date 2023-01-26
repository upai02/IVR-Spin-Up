#include "robot.h"
#include <vector>

void followPath(std::vector<std::vector<double>>& path, double lookForwardRadius, double translationalRPM, double maxRPM, double finalAngleDeg, bool printMessages);
void turnToAngle(double desiredAngleDeg, double toleranceDeg, bool debug, double p = 1.5);
void turnToPoint(double goalX = 0.45, double goalY = 0.45);
void moveMotors(double leftRPM, double rightRPM);