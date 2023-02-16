#include "drive.h"

double normalize_joystick(double input) {
  return input / 127.0;
}

const double sin_scale_factor = 2.9;
double sin_scale(double input) {
  return copysign(pow(sin((M_PI/2) * fabs(input)), sin_scale_factor), input);
}

double square_scale(double input) {
  return copysign(pow(input, 2), input);
}

// operator control drive
void op_drive() {
  drive_modes[drive_mode_idx].second();
}

void toggle_drive_mode() {
  drive_mode_idx = (drive_mode_idx + 1) % drive_modes.size();
}
std::string get_drive_name() {
  return drive_modes[drive_mode_idx].first;
}



// Regular tank drive with square scaling
void tank_drive() {
  double left = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
  double right = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
  left_side.move_voltage(left * 12000);
  right_side.move_voltage(right * 12000);
}
// Regular arcade drive with square scaling
void arcade_drive() {
  double forward = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
  double turn = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)));
  left_side.move_voltage((forward + turn) * 12000);
  right_side.move_voltage((forward - turn) * 12000);
}
// Hybrid arcade drive with square scaling
void hybrid_drive() {
  double forward = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
  double turn = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
  left_side.move_voltage((forward + turn) * 12000);
  right_side.move_voltage((forward - turn) * 12000);
}
// Dylan's Drive
void dylan_drive() {
  double left = sin_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
  double right = sin_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
  left_side.move_voltage(left * 12000);
  right_side.move_voltage(right * 12000);
}


// Special Prav Drive
const double kLowWheelNonLinearity = 0.5;
const double kLowNegInertiaThreshold = 0.65;
const double kLowNegInertiaTurnScalar = 0.875;
const double kLowNegInertiaCloseScalar = 1.0;
const double kLowNegInertiaFarScalar = 1.25;
const double kLowSensitivity = 0.65;
const double kQuickStopDeadband = 0.5;
const double kQuickStopWeight = 0.1;
const double kQuickStopScalar = 5.0;

double mOldWheel = 0.0;
double mQuickStopAccumlator = 0.0;
double mNegInertiaAccumlator = 0.0;
double limit_mag(double v, double mag) {
  return fmin(mag, fmax(-mag, v));
}
void constrain_range(double& var, double mag) {
  if (var > mag) var -= mag;
  else if (var < -mag) var += mag;
  else var = 0.0;
}
// complicated prav drive
void prav_drive() {
  double throttle = normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
  double wheel = normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

  double negInertia = wheel - mOldWheel;
  mOldWheel = wheel;

  // Apply sin curve to steering
  // can change this to my own sin scaling instead
  const double denominator = sin(M_PI/2.0 * kLowWheelNonLinearity);
  wheel = sin(M_PI/2.0 * kLowWheelNonLinearity * wheel) / denominator;
  wheel = sin(M_PI/2.0 * kLowWheelNonLinearity * wheel) / denominator;
  wheel = sin(M_PI/2.0 * kLowWheelNonLinearity * wheel) / denominator;

  double leftPwm, rightPwm, overPower, angularPower, linearPower, negInertiaScalar;
  // Negative inertia
  if (wheel * negInertia > 0) {
    // moving away from 0, get more wheel
    negInertiaScalar = kLowNegInertiaTurnScalar;
  } else {
    // moving towards 0, cut off wheel
    if (fabs(wheel) > kLowNegInertiaThreshold) {
      negInertiaScalar = kLowNegInertiaFarScalar;
    } else {
      negInertiaScalar = kLowNegInertiaCloseScalar;
    }
  }

  double negInertiaPower = negInertia * negInertiaScalar;
  mNegInertiaAccumlator += negInertiaPower;

  wheel = wheel + mNegInertiaAccumlator;
  // if (mNegInertiaAccumlator > 1) {
  //   mNegInertiaAccumlator -= 1;
  // } else if (mNegInertiaAccumlator < -1) {
  //   mNegInertiaAccumlator += 1;
  // } else {
  //   mNegInertiaAccumlator = 0;
  // }
  constrain_range(mNegInertiaAccumlator, 1.0);
  linearPower = throttle;

  // Quickturn
  bool isQuickTurn = fabs(throttle) < 0.08; // quick turn when turning in place
  if (isQuickTurn) {
    if (fabs(linearPower) < kQuickStopDeadband) {
      double alpha = kQuickStopWeight;
      mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator + alpha * limit_mag(wheel, 1.0) * kQuickStopScalar;
    }
    overPower = 1.0;
    angularPower = wheel;
  } else {
    overPower = 0.0;
    angularPower = fabs(throttle) * wheel * kLowSensitivity - mQuickStopAccumlator;
    // if (mQuickStopAccumlator > 1) {
    //   mQuickStopAccumlator -= 1;
    // } else if (mQuickStopAccumlator < -1) {
    //   mQuickStopAccumlator += 1;
    // } else {
    //   mQuickStopAccumlator = 0.0;
    // }
    constrain_range(mQuickStopAccumlator, 1.0);
  }

  rightPwm = leftPwm = linearPower;
  leftPwm += angularPower;
  rightPwm -= angularPower;
  if (leftPwm > 1.0) {
    rightPwm -= overPower * (leftPwm - 1.0);
    leftPwm = 1.0;
  } else if (rightPwm > 1.0) {
    leftPwm -= overPower * (rightPwm - 1.0);
    rightPwm = 1.0;
  } else if (leftPwm < -1.0) {
    rightPwm += overPower * (-1.0 - leftPwm);
    leftPwm = -1.0;
  } else if (rightPwm < -1.0) {
    leftPwm += overPower * (-1.0 - rightPwm);
    rightPwm = -1.0;
  }

  left_side.move_voltage(leftPwm * 12000);
  right_side.move_voltage(rightPwm * 12000);
}