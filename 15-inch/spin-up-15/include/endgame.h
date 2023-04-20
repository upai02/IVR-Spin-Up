#pragma once
#include "robot.h"

extern bool eg_deploy_piston_state;
extern bool string_release_piston_state;
extern int eg_timer;

void init_endgame();

void start_endgame_timer();

void deploy_endgame();
void release_string();

void toggle_eg_deploy_piston();
void toggle_string_release_piston();
void set_eg_deploy_piston(bool value);
void set_string_release_piston(bool value);