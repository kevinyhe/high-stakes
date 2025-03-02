#include "config.hpp"
#include "main.h"

void blue_awp()
{
    chassis->setPose(-55.5, 16, -180);

    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::RED);

    chassis->moveToPose(-60.5, 9.5, 208, 1200, {.lead = 0.05});
    chassis->waitUntilDone();
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);

    pros::delay(1000);
    arm.set_state(mechanism::ArmState::IDLE);

    chassis->moveToPose(-30.316, 20.7, -120, 3000, {.forwards = false, .lead = 0.15});
    // chassis->moveToPoint(-30.316, 19.29, 3000, {.forwards = false});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {

        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();

    intake_lift.extend();
    chassis->moveToPoint(-45, 4, 3000);

    chassis->waitUntilDone();
    intake.set_state(mechanism::IntakeState::HOOK);
    intake_lift.retract();
    pros::delay(400);

    // chassis->moveToPoint();
    // middle ring
    chassis->moveToPoint(-24, 48, 3000);

    // corner sweep
    // chassis->moveToPoint(-55.612, 60.476, 2000);
    // doinker.extend();
    // chassis->turnToHeading(-150, 2000, {.minSpeed = 72, .earlyExitRange = 10});
    // chassis->waitUntilDone();
    // doinker.retract();
    // chassis->moveToPoint(-60, -60, 1500);

    chassis->moveToPose(-2.917, 8.452, 105, 3000);
    pros::delay(2000);
    intake.set_state(mechanism::IntakeState::DISABLED);
    chassis->waitUntilDone();
    doinker.extend();
    pros::delay(200);

    chassis->moveToPoint(-24, 24, 2000, {.forwards = false});
    chassis->waitUntil(18);
    doinker.retract();
    chassis->moveToPoint(-12.5, 5, 2000);
    intake.set_state(mechanism::IntakeState::HOOK);

    chassis->moveToPoint(-36, 24, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 4});
    chassis->waitUntilDone();
    intake.set_state(mechanism::IntakeState::DISABLED);
    pros::delay(200);
    clamp.set_autoclamp(false);
    clamp.retract();
    
    chassis->moveToPoint(-40, 0, 2000, {.minSpeed = 20, .earlyExitRange = 4});
    chassis->moveToPose(-28, -20, -60, 2000, {.forwards = false});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        
        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();
    intake.set_state(mechanism::IntakeState::HOOK);
    
    chassis->moveToPoint(-24, -48, 2000, {.minSpeed = 20, .earlyExitRange = 2});
    chassis->moveToPoint(-17, -17, 2000, {.forwards = false});
    chassis->waitUntilDone();
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    intake.set_state(mechanism::IntakeState::DISABLED);


}