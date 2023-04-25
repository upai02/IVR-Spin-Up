#include <cmath>
#include <string>
#include "pros/llemu.hpp"
#include "robot.h"
#include "movement_helper.h"

int sgn(double number) {
    return (number >= 0) ? 1 : -1;
}

/**
Optimizes angle, only works for [-360, 360] input, outputs
[-180, 180]
*/
double optimizeAngle(double angle) {
    if (angle > 180) {
        angle -= 360;
    } else if (angle < -180) {
        angle += 360;
    }

    return angle;
}

double reverseAngle(double angle) {
    if (angle >= 180.0) {
        angle -= 180.0;
    } else {
        angle += 180.0;
    }

    return angle;
}

/**
Angle in degrees for [0, 360], you can also just pass in a point for dx and dy and it
will be compared relative to the origin.
*/
double lineAndAngleAngularDiff(double dx, double dy, double angleDegrees) {
    // dx, dy and not dy, dx because when the robot points towards positive y it's facing 0/360
    double segmentAngle = atan2(dx, dy) * 180 / M_PI;

    // change domain to [0, 360]
    if (segmentAngle < 0) segmentAngle += 360;

    double angularDiff = segmentAngle - angleDegrees;

    return optimizeAngle(angularDiff);
}

double getRotationalRPM(double desiredAngleDeg, bool reversed, double p) {
    if (reversed) {
        return optimizeAngle(desiredAngleDeg - (reverseAngle(imu.get_heading()))) * p;
    } else {
        return optimizeAngle(desiredAngleDeg - imu.get_heading()) * p;
    }
}

double getTranslationalRPM(double dist_to_goal_meters, double max_translational_rpm, double total_path_distance, double rpm_per_meter) { // 540
    double MIN_RPM = 40.0;
    if ((total_path_distance / 3) < dist_to_goal_meters) {
        // less than 2 thirds through path
        MIN_RPM = 125.0;
        // pros::lcd::set_text(7, "First 2/3");
    } else {
        // pros::lcd::set_text(7, "total dist: " + std::to_string(total_path_distance));
    }
    pros::lcd::set_text(7, "total dist: " + std::to_string(dist_to_goal_meters));
    // double NOT_TURNING_CONST = 20.0;
    // double INACCURACY_THRESHOLD = 0.75;
    // double left_avg = (left_front_mtr.get_actual_velocity() + left_back_bot_mtr.get_actual_velocity() + left_back_top_mtr.get_actual_velocity()) / 3;
    // double right_avg = (right_front_mtr.get_actual_velocity() + right_back_bot_mtr.get_actual_velocity() + right_back_top_mtr.get_actual_velocity()) / 3;
    // double desired_rpm = std::min(std::max(dist_to_goal_meters * rpm_per_meter, MIN_RPM), max_translational_rpm);
    // if ((std::abs(left_avg - right_avg) < NOT_TURNING_CONST) && (std::abs((left_avg + right_avg) / 2) < (desired_rpm) * INACCURACY_THRESHOLD) {
    //     // robot is not turning and going significantly less speed than it should then
    //     // so we should give it a boost
    //     // percent error calculation 
    // }
    return std::min(std::max(dist_to_goal_meters * rpm_per_meter, MIN_RPM), max_translational_rpm);
}

double calculate_distance_two_points(std::vector<double> point_one, std::vector<double> point_two) {
    return std::sqrt(std::pow(point_one.at(1) - point_two.at(1), 2) + std::pow(point_one.at(0) - point_two.at(0), 2));
}

std::vector<double> calculate_remaining_dist(std::vector<std::vector<double>>& path, bool ignore_last_point) {
    std::vector<double> distances(path.size());

    double sum_of_dists = 0.0;
    int times_ran = 0;

    // worried about segmentation fault in final loop run so I'm adding this here
    distances[distances.size() - 1] = 0.0;

    // loop through path in reverse order and append to distances in reverse order. Each loop add new dist to sum_of_dists then put that in distances
    // this will run thorugh the domain [initial i - 1, 0] (inclusive)
    for (size_t i = path.size() - ((ignore_last_point) ? 2 : 1); i-- > 0; ) {
        times_ran++;
        // pros::lcd::set_text(3, "i: " + std::to_string(i) + "  sum: " + std::to_string(sum_of_dists));
        sum_of_dists += calculate_distance_two_points(path.at(i), path.at(i + 1));
        distances[i] = sum_of_dists;
        // pros::lcd::set_text(4, "distances[i]: " + std::to_string(distances.at(i)) + " TR: " + std::to_string(times_ran));
    }

    return distances;
}

double calcGoalAngle(std::vector<double> vect) {
    double desiredAngle = atan2(GOAL_X - vect[0], GOAL_Y - vect[1]) * 180 / M_PI;
    if (desiredAngle < 0) desiredAngle += 360;

    return desiredAngle;
}