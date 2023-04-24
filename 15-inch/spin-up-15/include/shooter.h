#pragma once
#include "robot.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

extern int target_flywheel_rpm;

const int close_range_rpm = 310;
const int long_range_rpm = 370;

extern pros::Task flywheel_task;

extern bool flywheel_running;
extern bool soft_spinning;


void release_sequence();

void release_discs();
void run_flywheel();
void stop_flywheel();
void shoot_thread();

void activate_close_range();
void activate_long_range();
void set_flywheel_rpm(int rpm);

double get_flywheel_rpm();

void soft_spin();