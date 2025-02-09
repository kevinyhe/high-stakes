#include "main.h"
#include "config.hpp"

void red_ring_side()
{
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();

    chassis.setPose(-56, 40.5, -90);

    // mogo, early exit for smoother
    chassis.moveToPoint(-36.5, 31, 3000, {.forwards = false, .minSpeed = 110, .earlyExitRange = 6});
    chassis.moveToPose(-31.785, 28.712, -60, 3000, {.forwards = false, .lead = 0.1});
    chassis.waitUntil(8);
    pros::delay(400);
    clamp.extend();
    pros::delay(300);
    chassis.cancelMotion();
    intake.set_state(mechanism::IntakeState::HOOK);

    // top rings
    chassis.swingToPoint(-7.5, 39.597, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 72, .earlyExitRange = 20});
    // chassis.moveToPoint(-14.706, 39.597, 3000, {.minSpeed = 72, .earlyExitRange = 4});
    chassis.moveToPoint(-9.0, 40, 2000, {.minSpeed = 72, .earlyExitRange = 2});
    // chassis.moveToPoint(-9.5, 54, 2000, {.minSpeed = 72, .earlyExitRange = 4});
    chassis.moveToPoint(-24.961, 29.597, 3000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 4});
    
    // swing back for bottom ring
    chassis.moveToPoint(-24, 48, 2000, {.minSpeed = 72, .earlyExitRange = 4});
    // chassis.moveToPoint(-9.5, 45, 3000, {.minSpeed = 60, .earlyExitRange = 4});
    chassis.moveToPoint(-24, 34, 2000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 6});
    // move towards alliance stake
    // chassis.moveToPoint(-48, 16, 3000);
    chassis.moveToPose(-48, 5, 180, 4000);
    chassis.waitUntil(20);
    intake_lift.extend();
    pros::delay(2000);
    intake_lift.retract();
    chassis.moveToPose(-48, 20, 180, 2000, {.forwards = false});
    chassis.moveToPoint(-48, 7, 2000);
}
// void red_ring_side()
// {
//     auto &arm = mechanism::Arm::get_instance();
//     auto &intake = mechanism::Intake::get_instance();

//     chassis.setPose(-56, 40.5, -90);

//     // mogo, early exit for smoother
//     chassis.moveToPoint(-36.5, 32, 3000, {.forwards = false, .minSpeed = 110, .earlyExitRange = 6});
//     chassis.moveToPose(-31.785, 28.712, -60, 3000, {.forwards = false, .lead = 0.1});
//     chassis.waitUntil(8);
//     clamp.extend();
//     pros::delay(150);
//     chassis.cancelMotion();
//     intake.set_state(mechanism::IntakeState::HOOK);

//     // top rings
//     chassis.swingToPoint(-7.5, 39.597, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 72, .earlyExitRange = 20});
//     // chassis.moveToPoint(-14.706, 39.597, 3000, {.minSpeed = 72, .earlyExitRange = 4});
//     chassis.moveToPoint(-9.5, 40, 2000, {.minSpeed = 72, .earlyExitRange = 4});
//     chassis.moveToPoint(-9.5, 54, 2000, {.minSpeed = 72, .earlyExitRange = 4});
//     // chassis.moveToPoint(-6.2, 40, 3000, {.minSpeed = 60, .earlyExitRange });
//     chassis.moveToPoint(-24.961, 29.597, 3000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 4});

//     // swing back for bottom ring
//     chassis.moveToPoint(-24, 48, 2000, {.minSpeed = 72, .earlyExitRange = 4});
//     chassis.moveToPoint(-24, 34, 2000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 6});
//     // move towards alliance stake
//     // chassis.moveToPoint(-48, 16, 3000);
//     chassis.moveToPose(-48, 7, 180, 2000);
//     chassis.waitUntil(16);
//     clamp.retract();
//     intake_lift.extend();
//     chassis.moveToPose(-48, 15, 180, 2000);
//     intake_lift.retract();
//     doinker.extend();
//     chassis.waitUntil(10);
//     intake.set_state(mechanism::IntakeState::DISABLED);
//     chassis.turnToHeading(0, 2000);
//     chassis.moveToPoint(-48, 0, 3000);

//     chassis.turnToHeading(90, 2000, {.direction = AngularDirection::CW_CLOCKWISE});
//     chassis.moveToPoint(-65, 0, 2000, {.forwards=false});

//     // chassis.swingToPoint(-60, 0, lemlib::DriveSide::RIGHT, 2000, { .minSpeed = 72, .earlyExitRange = 50});

//     // score alliance stake
//     doinker.retract();
//     chassis.waitUntilDone();

//     // touch ladder
//     intake.set_state(mechanism::IntakeState::HOOK);
//     pros::delay(1000);

//     chassis.moveToPoint(-30, 0, 2000);
//     arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
//     chassis.turnToHeading(-80, 2000);
// }
void blue_ring_side()
{
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();

    chassis.setPose(56, 40.5, 90);

    // mogo, early exit for smoother
    chassis.moveToPoint(36.5, 32, 3000, {.forwards = false, .minSpeed = 110, .earlyExitRange = 6});
    chassis.moveToPose(31.785, 27.712, 60, 3000, {.forwards = false, .lead = 0.1});
    chassis.waitUntil(8);
    clamp.extend();
    pros::delay(150);
    chassis.cancelMotion();
    intake.set_state(mechanism::IntakeState::HOOK);

    // top rings
    chassis.swingToPoint(7.5, 39.597, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 72, .earlyExitRange = 20});
    // chassis.moveToPoint(-14.706, 39.597, 3000, {.minSpeed = 72, .earlyExitRange = 4});
    chassis.moveToPoint(7.5, 40, 2000, {.minSpeed = 72, .earlyExitRange = 4});
    // chassis.moveToPoint(-6.2, 40, 3000, {.minSpeed = 60, .earlyExitRange });
    chassis.moveToPoint(24.961, 29.597, 3000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 4});

    // swing back for bottom ring
    chassis.moveToPoint(28, 48, 2000, {.minSpeed = 72, .earlyExitRange = 4});
    chassis.moveToPoint(24, 34, 2000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 6});
    // move towards alliance stake
    chassis.moveToPoint(48, 16, 3000, {.minSpeed = 80, .earlyExitRange = 12});
    chassis.waitUntil(16);
    clamp.retract();
    chassis.moveToPoint(48, 7, 2000);
    intake_lift.extend();
    chassis.moveToPoint(48, 15, 2000);
    intake_lift.retract();
    doinker.extend();
    chassis.waitUntil(10);
    intake.set_state(mechanism::IntakeState::DISABLED);

    // chassis.swingToPoint(-60, 0, lemlib::DriveSide::RIGHT, 2000, { .minSpeed = 72, .earlyExitRange = 50});

    // score alliance stake
    chassis.moveToPose(65, 0, -90, 5000, {.forwards = false});
    doinker.retract();
    chassis.waitUntilDone();

    // touch ladder
    intake.set_state(mechanism::IntakeState::HOOK);

    chassis.moveToPoint(-30, 0, 2000);
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
}

void blue_ring_side_safe() {
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();

    chassis.setPose(56, 40.5, 90);

    // mogo, early exit for smoother
    chassis.moveToPoint(36.5, 32, 3000, {.forwards = false, .minSpeed = 110, .earlyExitRange = 6});
    chassis.moveToPose(31.785, 28.712, 60, 3000, {.forwards = false, .lead = 0.2});
    chassis.waitUntil(8);
    clamp.extend();
    pros::delay(150);
    chassis.cancelMotion();
    intake.set_state(mechanism::IntakeState::HOOK);

    chassis.moveToPoint(24, 48, 2000);
    chassis.moveToPoint(19, 19, 2000);
    chassis.waitUntilDone();
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
}
void red_ring_side_safe() {
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();

    chassis.setPose(-56, 40.5, -90);

    // mogo, early exit for smoother
    chassis.moveToPoint(-36.5, 32, 3000, {.forwards = false, .minSpeed = 110, .earlyExitRange = 6});
    chassis.moveToPose(-31.785, 28.712, -60, 3000, {.forwards = false, .lead = 0.2});
    chassis.waitUntil(8);
    clamp.extend();
    pros::delay(150);
    chassis.cancelMotion();
    intake.set_state(mechanism::IntakeState::HOOK);

    chassis.moveToPoint(-24, 48, 2000);
    chassis.moveToPoint(-19, 19, 2000);
    chassis.waitUntilDone();
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
}


void autonomous() {
    red_ring_side();
}