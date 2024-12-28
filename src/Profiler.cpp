#include "Profiler.hpp"
#include "lemlib/pose.hpp"
#include <cmath>
#include <mutex>
#include <sstream>

Profiler::Profiler(lemlib::Chassis* chassis, lemlib::Drivetrain* drivetrain, const float b, const float zeta,
                   const float scrubFactor)
    : chassis(chassis),
      drivetrain(drivetrain),
      b(b),
      zeta(zeta),
      scrubFactor(scrubFactor),
      t([this]() { loop(); }),
      isRunning(false) {};

void Profiler::loop() {
    lemlib::Pose pose = chassis->getPose(true, true);
    lemlib::Pose lastPose = pose;
    while (true) {
        std::uint32_t now = pros::millis();
        pose = chassis->getPose(true, true);

        if (std::lock_guard<pros::Mutex> lock(m); isRunning) {
            distTraveled += pose.distance(lastPose);
            int i = distTraveled / step;
            if (i >= vels.size() - 1) {
                drivetrain->leftMotors->move_velocity(0);
                drivetrain->rightMotors->move_velocity(0);
                isRunning = false;
                break;
            }

            float vel = vels[i];
            float angularVel = vels[i] * curvatures[i];
            lemlib::Pose desiredPose(desiredPoses[i]);
            float eX =
                (desiredPose.x - pose.x) * std::cos(pose.theta) + (desiredPose.y - pose.y) * std::sin(pose.theta);
            float eY =
                -(desiredPose.x - pose.x) * std::sin(pose.theta) + (desiredPose.y - pose.y) * std::cos(pose.theta);
            float eTheta = desiredPose.theta - pose.theta;

            // Convert to meters for RAMSETE
            vel *= 0.0254;
            eX *= 0.0254;
            eY *= 0.0254;

            float k = 2 * zeta * std::sqrt(angularVel * angularVel + b * vel * vel);
            float newVel = vel * std::cos(eTheta) + k * eX;
            // TODO: Taylor approx. near 0? (1-x^2/6)
            float newAngularVel = angularVel + k * eTheta + (b * vel * std::sin(eTheta) * eY) / eTheta;
            vel = newVel;
            angularVel = newAngularVel;

            // Convert back to in/s
            vel *= 39.3701;

            float leftVel = vel - angularVel * drivetrain->trackWidth / 2 * scrubFactor;
            float rightVel = vel + angularVel * drivetrain->trackWidth / 2 * scrubFactor;

            float leftVelRpm = leftVel / (std::numbers::pi * drivetrain->wheelDiameter) * 60;
            float rightVelRpm = rightVel / (std::numbers::pi * drivetrain->wheelDiameter) * 60;

            drivetrain->leftMotors->move_velocity(leftVelRpm);
            drivetrain->rightMotors->move_velocity(rightVelRpm);
        }
        lastPose = pose;
        pros::Task::delay_until(&now, 10);
    }
}

void Profiler::follow(const asset& path) {
    vels.clear();
    curvatures.clear();
    desiredPoses.clear();
    std::stringstream s;
    s << path.buf;
    s >> step;
    std::istringstream is;
    for (std::string line; std::getline(s, line);) {
        is.clear();
        is.str(line);

        float vel;
        float curvature;
        lemlib::Pose desiredPose(0, 0);
        is >> vel >> curvature >> desiredPose.x >> desiredPose.y >> desiredPose.theta;

        vels.push_back(vel);
        curvatures.push_back(curvature);
        desiredPoses.push_back(desiredPose);
    }
    std::lock_guard<pros::Mutex> lock(m);
    distTraveled = 0;
    isRunning = true;
};

void Profiler::waitUntil(float d) {
    if (std::lock_guard<pros::Mutex> lock(m); isRunning) {
        while (distTraveled < d) { pros::delay(10); }
    }
}

void Profiler::waitUntilDone() {
    std::lock_guard<pros::Mutex> lock(m);
    while (isRunning) { pros::delay(10); }
}
