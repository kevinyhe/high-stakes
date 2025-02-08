#include "main.h"
#include "config.hpp"

void autonomous()
{
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();

    chassis.setPose(-56, 40.5, -90);

    // mogo, early exit for smoother
    chassis.moveToPoint(-36.5, 31, 3000, {.forwards = false, .minSpeed = 110, .earlyExitRange = 4});
    chassis.moveToPose(-31.785, 28.712, -60, 3000, {.forwards = false, .lead = 0.2});
    chassis.waitUntil(8);
    clamp.extend();
    pros::delay(150);
    chassis.cancelMotion();
    intake.set_state(mechanism::IntakeState::HOOK);

    // top rings
    chassis.swingToPoint(-8.606, 39.597, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 72, .earlyExitRange = 20});
    chassis.moveToPoint(-8.606, 39.597, 3000, {.minSpeed = 72, .earlyExitRange = 4});
    chassis.moveToPoint(-7.5, 40, 2000, {.minSpeed = 72, .earlyExitRange = 4});
    chassis.moveToPoint(-7, 50, 3000, {.maxSpeed = 80, .earlyExitRange = 2});

    // swing back for bottom ring
    chassis.swingToPoint(-24, 48, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 72, .earlyExitRange = 50});
    chassis.moveToPoint(-24, 48, 2000);

    // move towards alliance stake
    chassis.moveToPoint(-44, 21, 3000, {.minSpeed = 80, .earlyExitRange = 12});
    chassis.waitUntil(16);
    clamp.retract();
    chassis.moveToPose(-48, 7, 180, 2000);
    intake_lift.extend();

    // score alliance stake
    chassis.moveToPose(-48, 15, 180, 2000, {.forwards = false});
    chassis.waitUntil(5);
    intake_lift.retract();
    intake.set_state(mechanism::IntakeState::DISABLED);
    chassis.moveToPose(-62, 0, 90, 3000, {.forwards = false});
    chassis.waitUntilDone();

    // touch ladder
    intake.set_state(mechanism::IntakeState::HOOK);

    chassis.moveToPoint(-36, 0, 2000);
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
}