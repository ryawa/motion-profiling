#include <chrono>
#include <cstdint>
#include <numbers>
#include <ratio>
#include <iostream>
#include <utility>
#include "main.h"
#include "lemlib/api.hpp"
#include "Eigen/Dense"
#include "Constraints.hpp"
#include "Path.hpp"
#include "Trajectory.hpp"
#include "Pose.hpp"

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

pros::MotorGroup leftMotors({-19, -18, -16}, pros::v5::MotorGears::blue);
pros::MotorGroup rightMotors({6, 10, 2}, pros::v5::MotorGears::blue);
pros::Imu imu(11);
pros::Rotation horizontalEnc(-17);
pros::Rotation verticalEnc(-15);
// TODO: tune offsets
lemlib::TrackingWheel horizTrackingWheel(&horizontalEnc, 2.0, 1.125);
lemlib::TrackingWheel vertTrackingWheel(&verticalEnc, 2.0, 1.25);
const float trackWidth = 12.1875;

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, trackWidth, lemlib::Omniwheel::NEW_325, 450, 2);

lemlib::OdomSensors sensors(&vertTrackingWheel, nullptr, &horizTrackingWheel, nullptr, &imu);

lemlib::ControllerSettings linearController(8.4, 0, 65, 0, 1, 250, 1, 250, 9);
lemlib::ControllerSettings angularController(2, 0, 13, 0, 0, 0, 0, 0, 0);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

const float gearRatio = 36.0 / 48.0;
const float wheelDiameter = 3.25;
const float b = 2.0;
const float zeta = 0.7;

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
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;

    // QuinticHermite qh {
    //     {0, 0},
    //     {30, 0},
    //     {72, 24},
    //     {0, 72},
    // };

    // QuinticHermite qh {
    //     {0, 0},
    //     {30, 30},
    //     {48, 0},
    //     {30, 30},
    // };

    Path* qh = new QuinticHermite {
        {0, 0},
        {10, 0},
        {72, 0},
        {10, 0},
    };

    // TODO: friction
    // Constraints constraints {100, 150, 100, 12.375, 1};
    Constraints constraints {75, 150, 150, 12.375, 9999};
    Trajectory trajectory {qh, constraints, 0.1};

    auto t1 = high_resolution_clock::now();
    trajectory.generate(3, 0, 0, 0);
    auto t2 = high_resolution_clock::now();
    duration<float, std::milli> deltat = t2 - t1;
    std::cout << "TIME: " << deltat.count() << "\n\n";

    float d = 0;

    // lemlib::PID leftPid(0.05, 0, 0.1);
    // lemlib::PID rightPid(0.05, 0, 0.1);
    lemlib::PID leftPid(0, 0, 0);
    lemlib::PID rightPid(0, 0, 0);
    float leftPidSum = 0.0;
    float rightPidSum = 0.0;

    std::cout << std::fixed << "\033[1mCopy this:\033[0m\n[";

    pros::Motor left(-19, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
    pros::Motor right(6, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

    float prevLeftVel = 0;
    float prevRightVel = 0;

    while (true) {
        // float dl = left.get_position() / 360 * std::numbers::pi * wheelDiameter * gearRatio;
        // float dr = right.get_position() / 360 * std::numbers::pi * wheelDiameter * gearRatio;
        // d = (dl + dr) / 2;
        // why less than y, euler integrate?
        d = (float)verticalEnc.get_position() / 100 / 360 * std::numbers::pi * 2.0;
        pros::lcd::print(1, "d: %lf", d);

        unsigned int i = d / trajectory.step;
        // if (i >= trajectory.vels.size()) { break; }
        i = std::min(i, trajectory.vels.size() - 1);
        std::uint32_t now = pros::millis();
        float vel = trajectory.vels[i];
        float angularVel = trajectory.vels[i] * trajectory.curvatures[i];
        Pose desiredPose = trajectory.desiredPoses[i];
        lemlib::Pose pose = chassis.getPose(true, true);
        float eX = (desiredPose.x - pose.x) * std::cos(pose.theta) + (desiredPose.y - pose.y) * std::sin(pose.theta);
        float eY = -(desiredPose.x - pose.x) * std::sin(pose.theta) + (desiredPose.y - pose.y) * std::cos(pose.theta);
        float eTheta = desiredPose.theta - pose.theta;
        pros::lcd::print(3, "%lf, %lf, %lf", pose.x, pose.y, pose.theta);
        pros::lcd::print(4, "%lf, %lf, %lf", eX, eY, eTheta);

        vel *= 0.0254;
        eX *= 0.0254;
        eY *= 0.0254;

        float k = 2 * zeta * std::sqrt(angularVel * angularVel + b * vel * vel);
        float newVel = vel * std::cos(eTheta) + k * eX;
        float newAngularVel = angularVel + k * eTheta + (b * vel * std::sin(eTheta) * eY) / eTheta;
        // vel = newVel;
        // angularVel = newAngularVel;

        vel *= 39.3701;

        float leftVel = vel - angularVel * constraints.trackWidth / 2;
        float rightVel = vel + angularVel * constraints.trackWidth / 2;

        float leftVelRpm = leftVel / (std::numbers::pi * wheelDiameter) * 60;
        float rightVelRpm = rightVel / (std::numbers::pi * wheelDiameter) * 60;
        std::pair<float, float> nextVels;
        if (i + 1 >= trajectory.vels.size()) {
            nextVels = std::make_pair(0, 0);
        } else {
            nextVels = trajectory.getWheelVelocities(i + 1, wheelDiameter);
        }
        // OR target - prev_target
        // (https://discord.com/channels/1094397185141002340/1094397185732390914/1316913821859708999)
        float leftAccel = leftVelRpm - prevLeftVel;
        float rightAccel = rightVelRpm - prevRightVel;
        prevLeftVel = leftVelRpm;
        prevRightVel = rightVelRpm;

        // kS, kP, turning scrub
        // Timing, trapezoidal sums, distance based on nearest point, euler integration
        const float kS = 10;
        const float kV = 0.208;
        const float kA = 2;

        // TODO: Filter vel?
        leftPidSum += leftPid.update(leftVelRpm - left.get_actual_velocity());
        rightPidSum += rightPid.update(rightVelRpm - right.get_actual_velocity());

        leftMotors.move(kS * sign(leftVelRpm) + kV * leftVelRpm + kA * leftAccel + leftPidSum);
        rightMotors.move(kS * sign(rightVelRpm) + kV * rightVelRpm + kA * rightAccel + rightPidSum);
        // leftMotors.move_velocity(leftVelRpm);
        // rightMotors.move_velocity(rightVelRpm);

        pros::lcd::print(5, "%lf %lf", leftVelRpm, rightVelRpm);

        log("Left desired", leftVelRpm, "Right desired", rightVelRpm, "Left actual", left.get_actual_velocity(),
            "Right actual", right.get_actual_velocity());
        // std::cout << "(" << chassis.getPose().x << "," << chassis.getPose().y << "),";

        // TODO: derivative scaling inaccurate, length of vels is too big
        pros::Task::delay_until(&now, 10);
    }

    std::cout << "\b]" << std::endl;
}
