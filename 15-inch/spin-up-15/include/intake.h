#pragma once
#include "robot.h"

// true = engaged, false = disengaged
extern bool mag_piston_state;

extern int discs_in_mag;
extern int last_disc_dist;
extern int rai_counter;

void init_intake();

void intake();
void outtake();

void toggle_mag_piston();
void set_mag_piston(bool value);

void reset_discs_in_mag();
int get_discs_in_mag();