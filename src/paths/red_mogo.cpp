#include "autonomous.hpp"
#include "config.hpp"
#include "main.h"

void red_mogo()
{
    chassis->setPose(-52, -31.5, 90);
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);

    // mogo rush
    chassis->moveToPoint(-14.5, -38.5, 3000, {.minSpeed = 20, .earlyExitRange = 4});
    chassis->waitUntilDone();
    // chassis->moveToPose(-17.5, -40, 105, 3000);
    doinker.extend();

    chassis->moveToPoint(-32.5, -36, 3000, {.forwards = false});
    chassis->waitUntil(15);
    doinker.retract();

    chassis->turnToPoint(-16.5, -46, 2000, {.forwards = false});
    chassis->moveToPoint(-16.5, -46, 3000, {.forwards = false, .minSpeed = 90});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    intake.set_state(mechanism::IntakeState::HOOK);

    // bottom ring
    chassis->swingToPoint(-24, -55, lemlib::DriveSide::LEFT, 2000);
    chassis->moveToPoint(-24, -55, 2000);

    chassis->swingToPoint(-42, -24, lemlib::DriveSide::RIGHT, 2000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis->moveToPoint(-42, -24, 3000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 4});
    chassis->waitUntilDone();
    clamp.set_autoclamp(false);
    clamp.retract();
    intake.set_state(mechanism::IntakeState::DISABLED);

    chassis->moveToPose(-24, -24, -90, 3000, {.forwards = false});
    chassis->waitUntil(10);
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();

    chassis->moveToPose(-9.261, -7.64, 45, 2000, {.minSpeed = 40, .earlyExitRange = 3});
    chassis->waitUntilDone();
    doinker.extend();

    chassis->moveToPoint(-24, -24, 2000);
}