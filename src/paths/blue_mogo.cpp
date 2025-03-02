#include "autonomous.hpp"
#include "config.hpp"
#include "main.h"

void blue_mogo()
{
    chassis->setPose(49, -35.25, -112);
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::RED);

    // intake ring on path
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    chassis->moveToPoint(17.085, -49.37, 2000, {.minSpeed = 127, .earlyExitRange = 7});
    chassis->waitUntilDone();
    doinker.extend();
    chassis->moveToPoint(34, -48, 5000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 4});
    chassis->waitUntilDone();
    doinker.retract();
    pros::delay(300);
    // chassis->turnToPoint(13.5, -33.25, 2000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 5});
    chassis->moveToPoint(13.5, -37.25, 2000, {.forwards = false, .maxSpeed = 80});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();

    intake.set_state(mechanism::IntakeState::HOOK);
    // chassis->moveToPoint(52, -52, 3000, {.minSpeed = 72, .earlyExitRange = 8});
    // chassis->waitUntil(24);
    chassis->moveToPose(57.749, -59.439, 120, 2000, {.lead = 0.1});
    doinker.extend();
    // chassis->waitUntilDone();
    chassis->turnToHeading(20, 2000, {.minSpeed = 72});
    chassis->waitUntilDone();
    doinker.retract();
    chassis->moveToPoint(65, -44, 1500);
    
    chassis->moveToPoint(52, -30, 1500, {.minSpeed = 100, .earlyExitRange = 4});
    chassis->waitUntilDone();
    clamp.set_autoclamp(false);
    clamp.retract();
    pros::delay(100);
    intake.enable_stop_next_ring();
    chassis->moveToPoint(48, -20, 2000, {.minSpeed = 100, .earlyExitRange = 4});
    
    // move backwards
    // chassis->swingToPoint(62.325, -62.518, lemlib::DriveSide::RIGHT, 2000, {.forwards= false, .minSpeed = 72, .earlyExitRange = 20});
    // chassis->moveToPoint(36, -48, 2000, {.minSpeed = 100, .earlyExitRange = 8});
    
    chassis->moveToPoint(24, -24, 2000, {.forwards = false, .maxSpeed = 80});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();
    intake.disable_stop_next_ring();
    intake.set_state(mechanism::IntakeState::HOOK);

    chassis->moveToPoint(38, -10, 2000, {.minSpeed = 72, .earlyExitRange = 4});
    chassis->moveToPose(54, 6, 45, 2000, {.maxSpeed = 70});
    chassis->moveToPoint(18, -48, 2000, {.maxSpeed  = 80});
}