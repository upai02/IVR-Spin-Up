#pragma once
#include "robot.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <string>

extern int target_flywheel_rpm;

const int close_range_rpm = 310;
const int long_range_rpm = 385;
const int overflow_rpm = 260;
extern bool overflow;

extern pros::Task flywheel_task;

extern bool flywheel_running;
extern bool soft_spinning;

extern bool angle_changer_state;

void init_shooter();

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

void toggle_angle_changer();

std::string get_rpm_state_string();