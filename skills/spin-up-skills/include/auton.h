#pragma once
#include "api.h"
#include "robot.h"
#include "autos/FollowXYPath.cpp"

void drivePID(double inches);
void shootPID(double rpm);
void followXYPath(FollowXYPath& xyPath);
