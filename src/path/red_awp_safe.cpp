#include "autonomous.hpp"
#include "config.hpp"
#include "main.h"

void red_awp_safe()
{
    chassis->setPose(-61.5, 13.6, 212.0);
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);
    // intiial alliance stake
    arm.set_rotation_value(65.7);
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    pros::delay(350);
    chassis->moveToPoint(-54.423, 24.824, 2000, {.forwards = false, .minSpeed = 127});
    chassis->waitUntil(1);
    chassis->cancelMotion();
    chassis->moveToPoint(-36.835, 22.867, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 5});
    arm.set_state(mechanism::ArmState::PRIME);

    // mogo
    chassis->moveToPoint(-26.504, 24.089, 2000, {.forwards = false, .maxSpeed = 85});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {

        pros::delay(10);
    }
    chassis->cancelMotion();
    clamp.extend();

    // top rings
    chassis->moveToPoint(-11.015, 36.2, 2000, {.minSpeed = 70, .earlyExitRange = 8});
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPose(-9.022, 50.571, 5, 2000, {.lead = 0.5, .minSpeed = 60});
    chassis->waitUntil(36);
    chassis->cancelMotion();

    // middle ring
    chassis->swingToPoint(-24, 48, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 60, .earlyExitRange = 50});
    chassis->moveToPoint(-24, 48, 2000, {.minSpeed = 20, .earlyExitRange = 6});

    // bottom middle ring
    chassis->moveToPoint(-48, 0, 2000, {.minSpeed = 127, .earlyExitRange = 14});
    chassis->waitUntilDone();
    clamp.set_autoclamp(false);
    clamp.retract();

    chassis->moveToPose(-48, 0, -135, 2000, {.lead = 0.1, .minSpeed = 60});
    intake_lift.retract();
}