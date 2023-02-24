#include "robot.h"
#include <vector>

#define GOAL_X 0.45
#define GOAL_Y 3.2

// void followPath(std::vector<std::vector<double>>& path, double lookForwardRadius, double translationalRPM, double maxRPM, double finalAngleDeg, bool reversed, bool printMessages);
void followPath(std::vector<std::vector<double>>& path, double finalAngleDeg, bool reversed, bool spinAtEnd = false, double lookForwardRadius = 0.5, double final_angle_tolerance_deg = 3.0, double translationalRPM = 150.0, double maxRPM = 200.0, bool printMessages = false);
void turnToAngle(double desiredAngleDeg, double toleranceDeg, bool debug = false, double p = 2.0);
// Calculate the angle needed to face the goal when at a given (future) position.
double calcGoalAngle(std::vector<double> vect);
void turnToPoint(double goalXCoordinate = GOAL_X, double goalYCoordinate = GOAL_Y);
void moveMotors(double leftRPM, double rightRPM);
void SmartStop();
void stopMotors();