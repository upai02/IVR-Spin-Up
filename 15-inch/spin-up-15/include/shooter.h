#pragma once
#include "robot.h"

extern int flywheel_rpm;

const int close_range_rpm = 300;
const int long_range_rpm = 550;

extern pros::Task flywheel_task;

extern bool flywheel_running;
extern bool soft_spinning;

void release_discs();
void run_flywheel();
void stop_flywheel();
void shoot_thread();

void activate_close_range();
void activate_long_range();
void set_flywheel_rpm(int rpm);

void soft_spin();