#include "main.h"
#include "config.hpp"

void red_mogo()
{
    chassis->setPose(-56, -31.5, 90);
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);

    // mogo rush
    chassis->moveToPoint(-14.5, -38.5, 3000, {.minSpeed = 20, .earlyExitRange = 4});
    // chassis->moveToPose(-17.5, -40, 105, 3000);
    chassis->waitUntilDone();
    doinker.extend();

    chassis->moveToPoint(-32.5, -32, 3000, {.forwards = false});
    chassis->waitUntil(12.5);
    doinker.retract();

    chassis->turnToPoint(-16.5, -46, 2000, {.forwards = false});
    chassis->moveToPoint(-16.5, -46, 3000, {.forwards = false, .minSpeed=90});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    intake.set_state(mechanism::IntakeState::HOOK);
    // bottom ring
    chassis->moveToPoint(-24, -55, 2000);
    
    chassis->swingToPoint(-48, -24, lemlib::DriveSide::RIGHT, 2000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis->moveToPoint(-48, -24, 3000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 4});
    chassis->waitUntilDone();
    clamp.set_autoclamp(false);
    clamp.retract();
    intake.set_state(mechanism::IntakeState::DISABLED);

    chassis->moveToPose(-26, -24, -90, 3000, {.forwards = false});
    chassis->waitUntil(10);
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    
}

void prog_skills() {
    chassis->setPose(-66, 0, 90);

    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();
    
    // alliance stake
    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(300);
    intake.set_state(mechanism::IntakeState::DISABLED);

    chassis->moveToPose(-48, 0, 0, 2000, {.maxSpeed = 110});
    chassis->turnToPoint(-48, -22.5, 2000 , {.forwards = false});
    chassis->moveToPose(-48, -22.5, 0, 2500, {.maxSpeed = 110});

    // TODO: clamp shit
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value()) {
        pros::delay(20);
    }

    chassis->cancelMotion();

    // first 2 rings
    chassis->moveToPoint(-21, -24, 2000, {.minSpeed = 127, .earlyExitRange = 2.5});
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPoint(0, -40, 2000, {.minSpeed = 127, .earlyExitRange = 6}); // to avoid ladder
    chassis->moveToPoint(25.5, -49, 3000, {.minSpeed = 90, .earlyExitRange = 4});
    
    // first blue stack
    chassis->moveToPoint(48, -60, 2000);
    chassis->waitUntil(10);
    arm.set_state(mechanism::ArmState::LOAD);

    chassis->moveToPoint(4.5, -44.5, 3000, {.minSpeed = 50, .earlyExitRange = 4});
    
    // wall stake
    chassis->moveToPose(0, -63, 180, 2500);
    arm.set_state(mechanism::ArmState::PRIME);
    chassis->waitUntilDone();
    intake.stop_next_ring();
    
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    pros::delay(800);

    // move back
    chassis->moveToPoint(0, -61, 1000, {.forwards = false});
    chassis->moveToPoint(0, -63, 2500);
    chassis->waitUntilDone();

    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
}

void autonomous()
{
    red_mogo();
}