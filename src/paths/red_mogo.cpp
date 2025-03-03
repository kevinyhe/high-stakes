#include "autonomous.hpp"
#include "config.hpp"
#include "main.h"

void red_mogo()
{
    chassis->setPose(-49, -57.5, 68);
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);

    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    chassis->moveToPoint(-16.829, -45.305, 2000, {.minSpeed = 127, .earlyExitRange = 7});

    // intake ring on path
    chassis->waitUntilDone();
    doinker.extend();
    chassis->moveToPoint(-34, -47, 5000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 2});
    chassis->waitUntilDone();
    doinker.retract();
    pros::delay(300);

    chassis->moveToPoint(-15.673, -52.857, 2000, {.forwards = false, .maxSpeed = 70});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    pros::delay(300);
    clamp.extend();

    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPoint(-40, -57, 3000, {.minSpeed = 72, .earlyExitRange = 8});
    chassis->waitUntil(24);
    doinker.extend();
    chassis->moveToPose(-60.899, -62.72, -150, 1500, {.lead = 0.1});
    pros::delay(1000);
    intake.set_state(mechanism::IntakeState::DISABLED);
    chassis->turnToHeading(140, 2000, {.minSpeed = 72, .earlyExitRange = 10});
    chassis->waitUntilDone();
    doinker.retract();

    intake.set_state(mechanism::IntakeState::HOOK);

    chassis->moveToPoint(-52, -65, 800);
    intake.enable_stop_next_ring();
    chassis->moveToPoint(-40, -48, 1500, {.minSpeed = 100, .earlyExitRange = 4});
    pros::delay(600);
    clamp.set_autoclamp(false);
    clamp.retract();
    
    chassis->moveToPoint(-24, -24, 2000, {.forwards = false, .maxSpeed = 90});
    intake.disable_stop_next_ring();
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    chassis->waitUntil(8);

    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();

    chassis->moveToPose(-8.412, -7.99, 50, 3000, {.lead = 0.2});
    chassis->waitUntilDone();
    doinker.extend();
    pros::delay(500);
    chassis->moveToPoint(-24, -24, 2000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 4});
   chassis->waitUntilDone();
    doinker.retract();
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPoint(-8, -24, 2000);
    // chassis->moveToPose(-54, 6, -45, 2000, {.maxSpeed = 70});
}