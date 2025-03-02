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
    chassis->moveToPoint(-34, -47, 5000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 4});
    chassis->waitUntilDone();
    doinker.retract();
    pros::delay(300);

    chassis->moveToPoint(-15.673, -55.857, 2000, {.forwards = false, .maxSpeed = 70});
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
    chassis->moveToPose(-60.899, -59.022, -150, 2000, {.lead = 0.1});
    intake.set_state(mechanism::IntakeState::DISABLED);
    chassis->turnToHeading(120, 2000, {.minSpeed = 72, .earlyExitRange = 10});
    chassis->waitUntilDone();
    doinker.retract();

    intake.set_state(mechanism::IntakeState::HOOK);

    chassis->moveToPoint(-52, -65, 800);
    chassis->moveToPoint(-36, -40, 1500, {.minSpeed = 100, .earlyExitRange = 4});
    pros::delay(600);
    clamp.set_autoclamp(false);
    clamp.retract();

    chassis->moveToPoint(-24, -24, 2000, {.forwards = false, .maxSpeed = 90});
    intake.set_state(mechanism::IntakeState::DISABLED);

    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();

    chassis->moveToPoint(-8.412, -10.99, 3000);
    chassis->waitUntilDone();
    doinker.extend();
    chassis->moveToPoint(-24, -24, 2000, {.forwards = false,.minSpeed = 72, .earlyExitRange = 4});
    chassis->moveToPoint(-38, -10, 2000, {.minSpeed = 72, .earlyExitRange = 4});
    doinker.retract();
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPose(-54, 6, 45, 2000, {.maxSpeed = 70});
}