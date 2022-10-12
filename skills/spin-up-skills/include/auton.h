#pragma once
#include "api.h"
#include "robot.h"
#include "autos/FollowXYPath.cpp"

void drivePID(double inches);

void followXYPath(FollowXYPath& xyPath);