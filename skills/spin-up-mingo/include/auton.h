#pragma once
#include "api.h"
#include "robot.h"
#include "autos/FollowXYPath.cpp"
#include <vector>

void drivePID(double inches);
void shootPID(double angle);
void followXYPath(FollowXYPath& xyPath);
void rollerAuto();
void boltSkillsAuto();
