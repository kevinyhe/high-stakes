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
    arm.set_state(mechanism::ArmState::IDLE);

    // mogo
    chassis->moveToPoint(-22.504, 25.589, 2000, {.forwards = false, .maxSpeed = 90});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {

        pros::delay(10);
    }
    chassis->cancelMotion();
    clamp.extend();

    // top rings
    chassis->moveToPoint(-11.515, 36.2, 2000, {.minSpeed = 80, .earlyExitRange = 8});
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPose(-8.722, 60.571, 0, 2000, {.lead = 0.4, .minSpeed = 60});
    chassis->waitUntil(21);
    chassis->cancelMotion();

    // middle ring
    chassis->swingToPoint(-22, 48, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 60, .earlyExitRange = 30});
    chassis->moveToPoint(-22, 48, 2000, {.minSpeed = 20, .earlyExitRange = 3});

    chassis->turnToPoint(-60, 60, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 127, .earlyExitRange = 60});
    arm.set_state(mechanism::ArmState::ALLIANCE_STAKE);

    // corner
    chassis->moveToPoint(-59.79, 60.278, 2000, {.minSpeed = 127, .earlyExitRange = 12});
    chassis->moveToPose(-62.79, 62.278, 330, 2000, {.lead = 0.1, .minSpeed = 70});

    pros::delay(400);
    chassis->moveToPoint(-50, 50, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 3});
    arm.set_state(mechanism::ArmState::PRIME);
    clamp.set_autoclamp(false);

    // alliance ring
    chassis->moveToPoint(-48, 15, 2000, {.minSpeed = 127, .earlyExitRange = 14});
    chassis->moveToPose(-48, 0, -180, 2000, {.lead = 0.1, .maxSpeed = 70, .minSpeed = 60});
    intake.enable_stop_next_ring();
    intake_lift.extend();
    clamp.retract();
    chassis->waitUntil(16);
    intake_lift.retract();

    // clamp
    chassis->turnToPoint(-24, -24, 2000, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 20, .earlyExitRange = 40});
    chassis->moveToPoint(-24, -24, 2000, {.forwards = false, .maxSpeed = 85});
    chassis->waitUntil(12);
    intake.disable_stop_next_ring();
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {

        pros::delay(10);
    }
    chassis->cancelMotion();
    clamp.extend();
    intake.set_state(mechanism::IntakeState::HOOK);

    // corner rings
    chassis->moveToPoint(-52, -35, 2000, {.minSpeed = 127, .earlyExitRange = 13});
    arm.set_state(mechanism::ArmState::ALLIANCE_STAKE);
    chassis->moveToPoint(-62.79, -63.278, 2000, {.minSpeed = 127, .earlyExitRange = 12});
    chassis->moveToPose(-65.79, -64.278, 200, 2000, {.lead = 0.1, .minSpeed = 60});

    pros::delay(400);

    chassis->moveToPoint(-47, -40, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 8});

    chassis->moveToPoint(-24, -48, 2000, {.minSpeed = 127, .earlyExitRange = 8});
    chassis->moveToPoint(-20, -20, 2000, {.minSpeed = 50});
    chassis->waitUntilDone();
    arm.set_state(mechanism::ArmState::ALLIANCE_STAKE);

    // TODO: check dejam bug
}