#include "config.hpp"
#include "main.h"

void red_ring_side()
{
    chassis->setPose(-55.5, -16, 0);

    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);

    chassis->moveToPose(-60.5, -9.5, -26, 1200, {.lead = 0.05});
    chassis->waitUntilDone();
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);

    pros::delay(1000);
    arm.set_state(mechanism::ArmState::IDLE);

    chassis->moveToPose(-30.316, -21.7, -60, 3000, {.forwards = false, .lead = 0.15});
    // chassis->moveToPoint(-30.316, 19.29, 3000, {.forwards = false});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {

        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();

    intake_lift.extend();
    chassis->moveToPoint(-43, -6, 3000);

    chassis->waitUntilDone();
    intake.set_state(mechanism::IntakeState::HOOK);
    intake_lift.retract();
    pros::delay(1000);

    // middle ring
    chassis->moveToPoint(-22.5, -48, 3000, {.minSpeed = 20, .earlyExitRange = 2});

    // corner sweep
    // chassis->moveToPoint(-55.612, 60.476, 2000);
    // doinker.extend();
    // chassis->turnToHeading(-150, 2000, {.minSpeed = 72, .earlyExitRange = 10});
    // chassis->waitUntilDone();
    // doinker.retract();
    // chassis->moveToPoint(-60, -60, 1500);

    // chassis->moveToPose(-6.917, -6.452, 60, 3000);
    // // pros::delay(2000);
    // intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    // chassis->waitUntilDone();
    // doinker.extend();
    // pros::delay(200);

    // chassis->moveToPoint(-24, -24, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 4});
    // chassis->waitUntilDone();
    // doinker.retract();
    // chassis->moveToPoint(-12.5, -5, 2000);
    // intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPoint(-24, -24, 3000, {.forwards = false});
    // chassis->moveToPoint(-11, 38, 3000, {.minSpeed = 20, .earlyExitRange = 2});
    // chassis->moveToPoint(-24, 24, 2000, {.forwards=se, .minSpeed = 20, .earlyExitRange = 2});
    chassis->moveToPose(-19, -19, 45, 4000);
    chassis->waitUntilDone();
    intake.set_state(mechanism::IntakeState::DISABLED);
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
}
 
// // negative
// void red_ring_side()
// {
//     chassis->setPose(-55.5, 16, -180);

//     auto &arm = mechanism::Arm::get_instance();
//     auto &intake = mechanism::Intake::get_instance();
//     auto &clamp = mechanism::Clamp::get_instance();

//     intake.enable_sort(mechanism::Intake::RingColours::RED);

//     chassis->moveToPose(-60.5, 9.5, 210, 1200, {.lead = 0.05});
//     chassis->waitUntilDone();
//     arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);

//     pros::delay(1000);
//     arm.set_state(mechanism::ArmState::IDLE);   

//     chassis->moveToPose(-25.316, 21.7, -120, 3000, {.forwards = false, .lead = 0.15});
//     // chassis->moveToPoint(-30.316, 19.29, 3000, {.forwards = false});
//     clamp.set_autoclamp(true);
//     while (chassis->isInMotion() && !clamp.get_value())
//     {

//         pros::delay(20);
//     }
//     chassis->cancelMotion();
//     clamp.extend();

//     intake_lift.extend();
//     chassis->moveToPoint(-43, 6, 3000);

//     chassis->waitUntilDone();
//     intake.set_state(mechanism::IntakeState::HOOK);
//     intake_lift.retract();
//     pros::delay(400);

//     // chassis->moveToPoint();
//     // middle ring
//     chassis->moveToPoint(-23, 48, 3000);

//     // corner sweep
//     // chassis->moveToPoint(-55.612, 60.476, 2000);
//     // doinker.extend();
//     // chassis->turnToHeading(-150, 2000, {.minSpeed = 72, .earlyExitRange = 10});
//     // chassis->waitUntilDone();
//     // doinker.retract();
//     // chassis->moveToPoint(-60, -60, 1500);

//     // chassis->moveToPose(-1.917, 9.452, 105, 3000);
//     // pros::delay(2000);
//     // intake.set_state(mechanism::IntakeState::FIRST_HOOK);
//     // chassis->waitUntilDone();
//     // doinker.extend();
//     // pros::delay(200);

//     // chassis->moveToPoint(-24, 24, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 4});
//     // chassis->waitUntilDone();
//     // doinker.retract();
//     // chassis->moveToPoint(-12.5, 5, 2000);
//     // intake.set_state(mechanism::IntakeState::HOOK);
//     // chassis->moveToPoint(-24, 24, 3000, {.forwards = false});
//     // chassis->moveToPoint(-11, 38, 3000, {.minSpeed = 20, .earlyExitRange = 2});
//     // chassis->moveToPoint(-24, 24, 2000, {.forwards=se, .minSpeed = 20, .earlyExitRange = 2});
//     chassis->moveToPose(-19, 19, 135, 4000);
//     chassis->waitUntilDone();
//     intake.set_state(mechanism::IntakeState::DISABLED);
//     arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
// }