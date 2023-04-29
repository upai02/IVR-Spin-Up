#pragma once
#include "api.h"
#include "robot.h"

#define NUM_INTS 6

// return variables from cv algorithm
extern int bbox_x1;
extern int bbox_x2;
extern int bbox_y1;
extern int bbox_y2;
extern int cx_offset;
extern int cy_offset;

// auto aim task
// extern pros::Task auto_aim_task;

// stores the values from the rasppi
extern char data_received [NUM_INTS * 10];
extern char previous_data_received [NUM_INTS * 10];
extern int rasppi_ret_vals [NUM_INTS];

void parse_data(int* read_out_values, char* read_in_str, int read_out_size, int read_in_str_size, int &num_elements);
bool read_from_pi();
void auto_aim();