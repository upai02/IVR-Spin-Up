#pragma once
#include "robot.h"
#include <string>

extern int drive_mode_idx;

void op_drive();
void toggle_drive_mode();
std::string get_drive_name();

void tank_drive();
void arcade_drive();
void hybrid_drive();
void dylan_drive();
void akap_drive();
void prav_drive();

extern std::vector<std::pair<std::string,std::function<void()>>> drive_modes;

// Utility functions
double normalize_joysticks(double input);
double sin_scale(double input);
double square_scale(double input);
