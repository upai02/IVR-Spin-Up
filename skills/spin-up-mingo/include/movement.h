#include "robot.h"
#include <vector>

void followPath(std::vector<std::vector<double>>& path, double lookForwardRadius, double translationalRPM, double maxRPM, double finalAngleDeg, bool printMessages);