#pragma once
#include "robot.h"

extern int discs_in_mag;
extern int last_disc_dist;
extern int rai_counter;

void init_intake();

void intake();
void outtake();

void reset_discs_in_mag();
int get_discs_in_mag();