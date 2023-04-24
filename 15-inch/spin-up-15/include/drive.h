#pragma once
#include "robot.h"
#include <string>

extern int drive_mode_idx;

void op_drive(); // Called in opcontrol
void toggle_drive_mode(); // toggles between drive modes(akaash and dylan)
std::string get_drive_name();

void tank_drive();
void arcade_drive();
void hybrid_drive();
void dylan_drive();
void akap_drive();


extern std::vector<std::pair<std::string,std::function<void()>>> drive_modes;

// Utility functions
double normalize_joysticks(double input);
double sin_scale(double input);
double square_scale(double input);
