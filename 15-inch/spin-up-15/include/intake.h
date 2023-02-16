#pragma once
#include "robot.h"

bool intake_out = true;
bool mag_down = true;

int discs_in_mag = 0;

void intake();
void outtake();

void toggle_intake_piston();
void toggle_mag_piston();

void reset_discs_in_mag();
int get_discs_in_mag();