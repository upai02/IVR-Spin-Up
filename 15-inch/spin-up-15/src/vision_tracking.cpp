#include "vision_tracking.h"

// initialize variables
int bbox_x1 = 0;
int bbox_x2 = 0;
int bbox_y1 = 0;
int bbox_y2 = 0;
int cx_offset = 0;
int cy_offset = 0;

char data_received [NUM_INTS * 10];
char previous_data_received [NUM_INTS * 10];
int rasppi_ret_vals [NUM_INTS];

pros::Task auto_aim_task(auto_aim);

// parses raw data and seperates by delimiter(,)
void parse_data(int* read_out_values, char* read_in_str, int read_out_size, int read_in_str_size, int &num_elements) {
	int read_in_idx = 0;
	char curr_char;
	int read_out_idx = 0;
	for (int i = 0; i < read_in_str_size; i++) {

		if (read_out_idx >= read_out_size)
			break;

		curr_char = read_in_str[i];

		if (curr_char == ',' || curr_char == '\n') {
			read_in_str[i] = '\n';
			read_out_values[read_out_idx] = std::stoi(&read_in_str[read_in_idx]);

			/* update counters */
			read_out_idx++;
			read_in_idx = i + 1;
			num_elements++;

			if (curr_char == ',')
				continue;
			
			/* the read_in_str_size may be greater than the actual amount of data available from serial port */
			if (curr_char == '\n')
				break;
		}
	}

	return;
}

// reads data from pi and stores it in the global variables
bool read_from_pi() {
  fgets(data_received, sizeof(data_received), stdin);

  if (strcmp(data_received, previous_data_received) != 0) {
    // pros::screen::print(TEXT_SMALL, 1, "%s", data_received);
    /* copy the data to the previous array */
    strcpy(previous_data_received, data_received);
    int num_elements;
    /* now use data_received to parse and return integer values */
    parse_data(rasppi_ret_vals, data_received, NUM_INTS, NUM_INTS*10, num_elements);
    bbox_x1 = rasppi_ret_vals[0];
    bbox_x2 = rasppi_ret_vals[1];
    bbox_y1 = rasppi_ret_vals[2];
    bbox_y2 = rasppi_ret_vals[3];
    cx_offset = rasppi_ret_vals[4];
    cy_offset = rasppi_ret_vals[5];
    return true;
  }
  return false;
}

// uses pid to bring cx_offset to 0
void auto_aim() {
	// currently using simple control loop
	const double kp = 0.8;
	const double ki = 0.05;

	double integral = 0;

	// read once to get current cx_offset
	while (!read_from_pi()) {
    pros::delay(20);
  }

	while (abs(cx_offset) > 5) {
		// update cx_offset and other values
		while (!read_from_pi()) {
      pros::delay(20);
    }
		// print values to the brain
		// pros::lcd::print(0, "cx_: %d, x1_: %d, x2_: %d", cx_, x1_, x2_);
		// pros::lcd::print(1, "cy_: %d, y1_: %d, y2_: %d", cy_, y1_, y2_);
		// pros::lcd::print(2, "area of bbox: %d", (x2_ - x1_)*(y2_ - y1_));
		
    // calculate turn power
		integral += cx_offset;
		double turn_power = kp * cx_offset + ki * integral;
		left_side.move(turn_power);
		right_side.move(-turn_power);
		
		pros::delay(20);
	}

}