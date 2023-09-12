#pragma once
#include "robot.h"

extern int discs_in_mag;
extern int last_disc_dist;
extern int rai_counter;


void intake();
void intake_auton();
void outtake();

void reset_discs_in_mag();
int get_discs_in_mag();