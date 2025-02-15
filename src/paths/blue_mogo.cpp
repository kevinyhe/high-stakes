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
    chassis->moveToPoint(-14.5, -37.5, 3000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis->waitUntilDone();
    // chassis->moveToPose(-17.5, -40, 105, 3000);
    doinker.extend();

    chassis->moveToPoint(-38.5, -30, 3000, {.forwards = false});
    chassis->waitUntil(18);
    doinker.retract();

    // first clamp
    // chassis->moveToPoint(-20.5, -43, 3000, {.forwards = false, .maxSpeed = 80});
    chassis->moveToPoint(-16.5, -45, 3000, {.forwards = false, .maxSpeed = 70});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPoint(-11, -37, 3000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 6});
    
    // bottom ring
    chassis->moveToPoint(-27, -53, 2000, {.minSpeed = 40, .earlyExitRange = 3});
    
    chassis->moveToPoint(-42, -24, 3000, {.minSpeed = 40, .earlyExitRange = 6});
    chassis->waitUntil(18);
    clamp.set_autoclamp(false);
    clamp.retract();
    intake.set_state(mechanism::IntakeState::DISABLED);
    
    chassis->moveToPoint(-16, -21, 3000, {.forwards = false, .maxSpeed = 70});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();
    pros::delay(200);

    chassis->moveToPose(-8.509, -8.57, 45, 2000);
    chassis->waitUntilDone();
    doinker.extend();
    pros::delay(200);

    chassis->moveToPoint(-15, -24, 2000, {.forwards = false});
    chassis->waitUntil(10);
    doinker.retract();
    chassis->moveToPoint(-12.5, -11, 2000);
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPose(-48, 0, -30, 3000);
}