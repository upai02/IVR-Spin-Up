#pragma once
#include "robot.h"

int flywheel_voltage = 9000;
const int close_range_voltage = 9000;
const int long_range_voltage = 12000;

void release_discs();
void run_flywheel();

void activate_close_range();
void activate_long_range();