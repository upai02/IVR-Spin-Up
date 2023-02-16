#pragma once
#include "robot.h"
#include <string>
#include "misc/rasppi_comms.h"
#include <vector>

using std::string;
using std::vector;


extern int flywheel_voltage;
const int close_range_voltage = 9000;
const int long_range_voltage = 12000;

void release_discs();
void run_flywheel();

void activate_close_range();
void activate_long_range();

// camera tracking stuff
void auto_aim();
void start_algo();
void end_algo();
void get_coords(vector<double> &coords, char *buffer, int &coords_idx, RasppiComms &comms);
double get_error_from_coords(vector<double>& coords);
