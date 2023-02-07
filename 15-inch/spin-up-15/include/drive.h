#pragma once
#include "robot.h"
#include <string>

std::vector<std::pair<std::string,std::function<void()>>> drive_modes = {{"Dylan_Drive", dylan_drive}, {"Prav_Drive", prav_drive}, {"Tank_Drive", tank_drive}, {"Arcade_Drive", arcade_drive}, {"Hybrid_Drive", hybrid_drive}};
int drive_mode_idx = 0;

void op_drive();
void toggle_drive_mode();
std::string get_drive_name();

void tank_drive();
void arcade_drive();
void hybrid_drive();
void dylan_drive();
void prav_drive();

// Utility functions
double normalize_joysticks(double input);
double sin_scale(double input);
double square_scale(double input);
