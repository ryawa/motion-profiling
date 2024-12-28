#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <ratio>
#include <iostream>
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "Eigen/Dense"
#include "Constraints.hpp"
#include "Path.hpp"
#include "Trajectory.hpp"
#include "Pose.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "Profiler.hpp"

ASSET(path_txt);

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

const float trackWidth = 10.8125;
const float scrubFactor = 1.25;
const float wheelDiameter = 2.75;

// const float kS = 10;
// const float kV = 0.18;
// const float kA = 3;

const float b = 2.0;
const float zeta = 0.7;

pros::MotorGroup leftMotors({-4, -6, -11}, pros::v5::MotorGears::blue);
pros::MotorGroup rightMotors({5, 7, 13}, pros::v5::MotorGears::blue);
pros::Imu imu(20);
pros::Rotation horizontalEnc(3);
pros::Rotation verticalEnc(2);
// TODO: tune offsets
lemlib::TrackingWheel horizTrackingWheel(&horizontalEnc, 2.0, 0.375);
lemlib::TrackingWheel vertTrackingWheel(&verticalEnc, 2.0, -0.84375);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, trackWidth, lemlib::Omniwheel::NEW_275, 450, 2);

lemlib::OdomSensors sensors(&vertTrackingWheel, nullptr, &horizTrackingWheel, nullptr, &imu);

lemlib::ControllerSettings linearController(8.4, 0, 65, 0, 1, 250, 1, 250, 9);
lemlib::ControllerSettings angularController(2, 0, 13, 0, 0, 0, 0, 0, 0);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

Profiler profiler(&chassis, &drivetrain, 2.0, 0.7, 1.25);

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    chassis.setPose(0, 0, 90);

    // leftMotors.set_brake_mode_all(pros::MotorBrake::hold);
    // rightMotors.set_brake_mode_all(pros::MotorBrake::hold);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

double sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

void opcontrol() {
    profiler.follow(path_txt);
    profiler.waitUntilDone();
    chassis.turnToHeading(0, 1000);

    // using std::chrono::duration;
    // using std::chrono::duration_cast;
    // using std::chrono::high_resolution_clock;

    // Path* qh = new QuinticHermite {
    //     {0, 0},
    //     {30, 0},
    //     {72, 72},
    //     {0, 180},
    // };

    // // QuinticHermite qh {
    // //     {0, 0},
    // //     {30, 30},
    // //     {48, 0},
    // //     {30, 30},
    // // };

    // // Path* qh = new QuinticHermite {
    // //     {0, 0},
    // //     {10, 0},
    // //     {72, 0},
    // //     {10, 0},
    // // };

    // // TODO: friction, speed up vel/acc
    // Constraints constraints {75, 100, 50, trackWidth, 0.02};
    // Trajectory trajectory {qh, constraints, 0.1};

    // auto t1 = high_resolution_clock::now();
    // trajectory.generate(3, 0, 0, 0);
    // auto t2 = high_resolution_clock::now();
    // duration<float, std::milli> deltat = t2 - t1;
    // std::cout << "TIME: " << deltat.count() << "\n\n";
    // std::cout << path_txt.buf;

    // float d = 0;

    // // 0.05, 0.1
    // lemlib::PID leftPid(0.07, 0, 0.1);
    // lemlib::PID rightPid(0.07, 0, 0.1);
    // float leftPidSum = 0.0;
    // float rightPidSum = 0.0;

    // std::cout << std::fixed << "\033[1mCopy this:\033[0m\n[";

    // // pros::Motor left(-19, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
    // // pros::Motor right(6, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

    // float prevLeftVel = 0;
    // float prevRightVel = 0;
    // float prevLeftError = 0;
    // float prevRightError = 0;

    // while (true) {
    //     // Integrate instead?
    //     d = (float)verticalEnc.get_position() / 100 / 360 * std::numbers::pi * 2.0;
    //     pros::lcd::print(1, "d: %lf", d);

    //     unsigned int i = d / trajectory.step;
    //     // if (i >= trajectory.vels.size()) { break; }
    //     i = std::min(i, trajectory.vels.size() - 1);
    //     std::uint32_t now = pros::millis();
    //     float vel = trajectory.vels[i];
    //     float angularVel = trajectory.vels[i] * trajectory.curvatures[i];
    //     Pose desiredPose = trajectory.desiredPoses[i];
    //     lemlib::Pose pose = chassis.getPose(true, true);
    //     float eX = (desiredPose.x - pose.x) * std::cos(pose.theta) + (desiredPose.y - pose.y) * std::sin(pose.theta);
    //     float eY = -(desiredPose.x - pose.x) * std::sin(pose.theta) + (desiredPose.y - pose.y) *
    //     std::cos(pose.theta); float eTheta = desiredPose.theta - pose.theta; pros::lcd::print(3, "%lf, %lf, %lf",
    //     pose.x, pose.y, pose.theta); pros::lcd::print(4, "%lf, %lf, %lf", eX, eY, eTheta);

    //     vel *= 0.0254;
    //     eX *= 0.0254;
    //     eY *= 0.0254;

    //     float k = 2 * zeta * std::sqrt(angularVel * angularVel + b * vel * vel);
    //     float newVel = vel * std::cos(eTheta) + k * eX;
    //     // TODO: Taylor approx. near 0? (1-x^2/6)
    //     float newAngularVel = angularVel + k * eTheta + (b * vel * std::sin(eTheta) * eY) / eTheta;
    //     vel = newVel;
    //     angularVel = newAngularVel;

    //     vel *= 39.3701;

    //     float leftVel = vel - angularVel * trackWidth / 2 * scrubFactor;
    //     float rightVel = vel + angularVel * trackWidth / 2 * scrubFactor;

    //     float leftVelRpm = leftVel / (std::numbers::pi * wheelDiameter) * 60;
    //     float rightVelRpm = rightVel / (std::numbers::pi * wheelDiameter) * 60;
    //     float leftAccel = leftVelRpm - prevLeftVel;
    //     float rightAccel = rightVelRpm - prevRightVel;
    //     prevLeftVel = leftVelRpm;
    //     prevRightVel = rightVelRpm;

    //     // Timing, trapezoidal sums, distance based on nearest point

    //     // TODO: Filter vel? (https://sylvie.fyi/sylib/docs/db/d8e/md_module_writeups__velocity__estimation.html)
    //     // float leftError = leftVelRpm - left.get_actual_velocity();
    //     // float rightError = rightVelRpm - right.get_actual_velocity();
    //     // leftPidSum += leftPid.update(leftError);
    //     // rightPidSum += rightPid.update(rightError);
    //     // if (std::signbit(leftError) != std::signbit(prevLeftError)) { leftPidSum = 0; }
    //     // if (std::signbit(rightError) != std::signbit(prevRightError)) { rightPidSum = 0; }
    //     // prevLeftError = leftError;
    //     // prevRightError = rightError;

    //     // leftMotors.move(kS * sign(leftVelRpm) + kV * leftVelRpm + kA * leftAccel + leftPidSum);
    //     // rightMotors.move(kS * sign(rightVelRpm) + kV * rightVelRpm + kA * rightAccel + rightPidSum);

    //     leftMotors.move_velocity(leftVelRpm);
    //     rightMotors.move_velocity(rightVelRpm);

    //     pros::lcd::print(5, "%lf %lf", leftVelRpm, rightVelRpm);

    //     // log("Left desired", leftVelRpm, "Right desired", rightVelRpm, "Left actual", left.get_actual_velocity(),
    //     //     "Right actual", right.get_actual_velocity());
    //     // std::cout << "(" << chassis.getPose().x << "," << chassis.getPose().y << "),";

    //     pros::Task::delay_until(&now, 10);
    // }

    // std::cout << "\b]" << std::endl;
}
