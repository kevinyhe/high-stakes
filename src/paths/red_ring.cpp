#include "config.hpp"
#include "main.h"

void red_ring()
{
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    chassis->setPose(-56, 40.5, -90);

    // mogo, early exit for smoother
    chassis->moveToPoint(-36.5, 31, 3000, {.forwards = false, .minSpeed = 110, .earlyExitRange = 6});
    chassis->moveToPose(-31.785, 28.712, -60, 3000, {.forwards = false, .lead = 0.1});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    intake.set_state(mechanism::IntakeState::HOOK);

    // top rings
    chassis->swingToPoint(-7.5, 39.597, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 72, .earlyExitRange = 20});
    // chassis->moveToPoint(-14.706, 39.597, 3000, {.minSpeed = 72, .earlyExitRange = 4});
    chassis->moveToPoint(-9.0, 40, 2000, {.minSpeed = 72, .earlyExitRange = 2});
    chassis->moveToPoint(-9.5, 54, 2000, {.minSpeed = 72, .earlyExitRange = 4});
    chassis->moveToPoint(-24.961, 29.597, 3000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 4});

    // swing back for bottom ring
    chassis->moveToPoint(-24, 48, 2000, {.minSpeed = 72, .earlyExitRange = 4});
    // chassis->moveToPoint(-9.5, 45, 3000, {.minSpeed = 60, .earlyExitRange = 4});
    chassis->moveToPoint(-24, 34, 2000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 6});
    // move towards alliance stake
    // chassis->moveToPoint(-48, 16, 3000);
    chassis->moveToPose(-48, 5, 180, 4000);
    chassis->waitUntil(20);
    intake_lift.extend();
    pros::delay(2000);
    intake_lift.retract();
    chassis->moveToPose(-48, 20, 180, 2000, {.forwards = false});
    chassis->moveToPoint(-48, 7, 2000);
}