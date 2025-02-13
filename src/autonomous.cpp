#include "main.h"
#include "config.hpp"

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
    while (chassis->isInMotion() && !clamp.is_clamped()) {
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
    prog_skills();
}