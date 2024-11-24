#include <chrono>
#include <cstdint>
#include <numbers>
#include <ratio>
#include <iostream>
#include "main.h"
#include "lemlib/api.hpp"
#include "Eigen/Dense"
#include "Constraints.hpp"
#include "Path.hpp"
#include "Trajectory.hpp"

template <typename T> void log_helper(const T& value) { std::cout << value; }

template <typename T, typename U, typename... Args> void log_helper(const T& key, const U& value, const Args&... args) {
    std::cout << key << "=" << value;
    if constexpr (sizeof...(args) > 0) {
        std::cout << ",";
        log_helper(args...);
    }
}

template <typename... Args> void log(const Args&... args) {
    std::cout << "VEX_DASHBOARD_BEGIN" << pros::millis() << ",";
    log_helper(args...);
    std::cout << "VEX_DASHBOARD_END" << std::flush;
}

pros::Rotation verticalEnc(-15);

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;

    // QuinticHermite qh {
    //     {0, 0},
    //     {30, 0},
    //     {72, 24},
    //     {30, 30},
    // };

    // QuinticHermite qh {
    //     {0, 0},
    //     {30, 30},
    //     {48, 0},
    //     {30, 30},
    // };

    QuinticHermite qh {
        {0, 0},
        {10, 0},
        {72, 0},
        {10, 0},
    };

    // TODO: Tune constraints
    Constraints constraints {100, 200, 200, 12.1875, 1};
    // TODO: Step?
    Trajectory trajectory {qh, constraints, 1};

    auto t1 = high_resolution_clock::now();
    trajectory.generate(3, 0, 0, 0);
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> dt = t2 - t1;
    std::cout << "TIME: " << dt.count() << "\n\n";

    std::cout << trajectory.vels.size() << " " << trajectory.angularVels.size() << " " << trajectory.accels.size()
              << " " << trajectory.angularAccels.size() << "\n\n";

    for (int i = 0; i < trajectory.vels.size(); i++) { std::cout << trajectory.vels[i] << " "; }
    std::cout << "\n\n";
    for (int i = 0; i < trajectory.angularVels.size(); i++) { std::cout << trajectory.angularVels[i] << " "; }
    std::cout << "\n\n";
    for (int i = 0; i < trajectory.accels.size(); i++) { std::cout << trajectory.accels[i] << " "; }
    std::cout << "\n\n";
    for (int i = 0; i < trajectory.angularAccels.size(); i++) { std::cout << trajectory.angularAccels[i] << " "; }
    std::cout << "\n\n";

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::MotorGroup left_mg({-19, -18, -16}, pros::v5::MotorGears::blue);
    pros::MotorGroup right_mg({6, 10, 2}, pros::v5::MotorGears::blue);
    left_mg.set_brake_mode_all(pros::MotorBrake::hold);
    right_mg.set_brake_mode_all(pros::MotorBrake::hold);
    double d = 0;

    // TODO: Tune PID
    lemlib::PID leftPid(0, 0, 0);
    lemlib::PID rightPid(0, 0, 0);
    double leftPidSum = 0.0;
    double rightPidSum = 0.0;

    while (true) {
        pros::Motor left(-19, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor right(6, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        double gearRatio = 36.0 / 48.0;
        double wheelDiameter = 3.25;
        double dl = left.get_position() / 360 * std::numbers::pi * wheelDiameter * gearRatio;
        double dr = right.get_position() / 360 * std::numbers::pi * wheelDiameter * gearRatio;
        d = (dl + dr) / 2;
        // TODO: odom?
        // d = (double)verticalEnc.get_position() / 100 / 360 * std::numbers::pi * 2.0;
        pros::lcd::print(1, "d: %lf", d);

        int i = d / trajectory.step;
        std::uint32_t now = pros::millis();
        if (i >= trajectory.vels.size()) {
            left_mg.move_velocity(0);
            right_mg.move_velocity(0);
        } else {
            double vel = trajectory.vels[i];
            double angularVel = trajectory.angularVels[i];
            double accel = trajectory.accels[i];
            double angularAccel = trajectory.angularAccels[i];

            double leftVel = vel - angularVel * constraints.trackWidth / 2;
            double rightVel = vel + angularVel * constraints.trackWidth / 2;
            double leftAccel = accel - angularAccel * constraints.trackWidth / 2;
            double rightAccel = accel + angularAccel * constraints.trackWidth / 2;

            // 600 rpm
            double leftVelRpm = leftVel / (std::numbers::pi * wheelDiameter) * 60;
            double rightVelRpm = rightVel / (std::numbers::pi * wheelDiameter) * 60;
            double leftAccelRpm = leftAccel / (std::numbers::pi * wheelDiameter) * 60;
            double rightAccelRpm = rightAccel / (std::numbers::pi * wheelDiameter) * 60;

            double kV = 1.145;
            double kA = 0.153;

            // TODO: Filter vel?
            leftPidSum += leftPid.update(leftVelRpm - left.get_actual_velocity());
            rightPidSum += rightPid.update(rightVelRpm - right.get_actual_velocity());

            left_mg.move(kV * leftVelRpm + kA * leftAccelRpm + leftPidSum);
            right_mg.move(kV * rightVelRpm + kA * rightAccelRpm + rightPidSum);

            log("Left desired", leftVelRpm, "Right desired", rightVelRpm, "Left actual", left.get_actual_velocity(),
                "Right actual", right.get_actual_velocity());
        }

        // TODO: derivative scaling inaccurate, length of vels is too big
        pros::Task::delay_until(&now, 10);
    }
}
