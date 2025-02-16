#include "config.hpp"
#include "autonomous.hpp"
#include "main.h"

void prog_skills()
{
    chassis->setPose(-65.5, 0, 90);

    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    // alliance stake
    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(1000);
    intake.set_state(mechanism::IntakeState::DISABLED);

    chassis->moveToPoint(-48, 0, 4000, {.maxSpeed = 110});
    chassis->turnToPoint(-48, -24, 4000, {.forwards = false});
    chassis->moveToPoint(-48, -24, 4000, {.forwards = false, .maxSpeed = 80});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    intake.set_state(mechanism::IntakeState::HOOK);

    chassis->moveToPoint(-48, -48, 3000);
    chassis->moveToPoint(-48, -60, 3000);
    chassis->turnToPoint(-62, -62, 3000, {.forwards = false});
    chassis->moveToPoint(-62, -62, 3000, {.forwards = false});
    chassis->waitUntilDone();
    intake.set_state(mechanism::IntakeState::DISABLED);

    clamp.set_autoclamp(false);
    clamp.retract();

    chassis->moveToPoint(-48, -48, 3000);
    chassis->turnToPoint(-48, 24, 2000);

    chassis->moveToPoint(-48, 24, 7000, {.forwards = false, .maxSpeed = 100});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    intake.set_state(mechanism::IntakeState::HOOK);

    chassis->moveToPoint(-48, 48, 3000);
    chassis->moveToPoint(-48, 60, 3000);

    chassis->turnToPoint(-62, 62, 3000, {.forwards = false});
    chassis->moveToPoint(-62, 62, 3000, {.forwards = false});
    chassis->waitUntilDone();
    intake.set_state(mechanism::IntakeState::DISABLED);

    clamp.set_autoclamp(false);
    clamp.retract();

    chassis->moveToPoint(0, 48, 5000);
    chassis->moveToPoint(57, 0, 5000);
    chassis->moveToPoint(62, 62, 5000);
    chassis->swingToPoint(72, 64, lemlib::DriveSide::RIGHT, 2000);

    chassis->moveToPoint(57, 0, 5000, {.forwards = false});
    chassis->moveToPoint(62, -62, 5000);
    chassis->swingToPoint(72, -64, lemlib::DriveSide::LEFT, 2000);
    chassis->moveToPoint(38, -38, 3000, {.forwards = false});
}
