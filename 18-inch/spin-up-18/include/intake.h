#pragma once
#include "robot.h"

extern bool intake_out;
extern bool mag_down;

extern int discs_in_mag;

void intake();
void outtake();

void toggle_intake_piston();
void toggle_mag_piston();

void reset_discs_in_mag();
int get_discs_in_mag();