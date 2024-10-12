#include <chrono>
#include <numbers>
#include <ratio>
#include "main.h"
#include "Eigen/Dense"
#include "Constraints.hpp"
#include "Path.hpp"
#include "Trajectory.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;

    // TODO: does v direction matter?
    QuinticHermite qh {
        {0, 0},
        {10, 0},
        {48, 0},
        {-10, 0},
    };

    // TODO: test friction coeff
    Constraints constraints {100, 100, 100, 12.625, 99999};
    Trajectory trajectory {qh, constraints, 0.1};

    auto t1 = high_resolution_clock::now();
    // TODO: startVel needs to be so high?
    trajectory.generate(5, 0.01, 0, 0);
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> dt = t2 - t1;
    std::cout << "TIME: " << dt.count() << "\n\n";

    // std::cout << trajectory.vels.size() << " " << trajectory.angularVels.size() << "\n\n";
    // for (int i = 0; i < trajectory.vels.size(); i++) { std::cout << trajectory.vels[i] << "\n"; }
    // for (int i = 0; i < trajectory.angularVels.size(); i++) { std::cout << trajectory.angularVels[i] << "\n"; }

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    // pros::MotorGroup left_mg({-1, -8, -10}, pros::v5::MotorGears::blue);
    // pros::MotorGroup right_mg({3, 5, 6}, pros::v5::MotorGears::blue);

    pros::MotorGroup left_mg({1, 8, 10}, pros::v5::MotorGears::blue);
    pros::MotorGroup right_mg({-3, -5, -6}, pros::v5::MotorGears::blue);
    double d = 0;

    while (true) {
        pros::Motor left(1, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor right(-6, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        double gearRatio = 36.0 / 48.0;
        double wheelDiameter = 3.25;
        double dl = left.get_position() / 360 * std::numbers::pi * wheelDiameter * gearRatio;
        double dr = right.get_position() / 360 * std::numbers::pi * wheelDiameter * gearRatio;
        d = (dl + dr) / 2;
        std::cout << "d: " << d << "\n";
        pros::lcd::print(1, "d: %lf", d);

        int i = d / trajectory.step;
        if (i > trajectory.vels.size()) {
            left_mg.move(0);
            right_mg.move(0);
        } else {
            double vel = trajectory.vels[i];
            double angularVel = trajectory.angularVels[i];
            double leftVel = vel - angularVel * constraints.trackWidth / 2;
            double rightVel = vel + angularVel * constraints.trackWidth / 2;
            // 600 rpm
            double leftWheel = leftVel / (std::numbers::pi * wheelDiameter) * 60;
            double rightWheel = rightVel / (std::numbers::pi * wheelDiameter) * 60;
            left_mg.move_velocity(leftWheel);
            right_mg.move_velocity(rightWheel);
            std::cout << "leftWheel: " << leftWheel << " " << "rightWheel: " << rightWheel << "\n";
        }

        pros::delay(20); // Run for 20 ms then update
    }
}
