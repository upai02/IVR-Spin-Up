#pragma once
#include "robot.h"

extern bool intake_out;
extern bool mag_down;

extern int discs_in_mag;
extern int last_disc_dist;
extern int rai_counter;

void intake();
void outtake();

void toggle_intake_piston();
void toggle_mag_piston();

void set_intake_piston(bool value);
void set_mag_piston(bool value);

void reset_discs_in_mag();
int get_discs_in_mag();