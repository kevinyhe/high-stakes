#include "autonomous.hpp"
#include "config.hpp"
#include "main.h"

void red_awp()
{
    chassis->setPose(-61.5, 13.6, 212.0);
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);
    // intiial alliance stake
    arm.set_rotation_value(65.7);
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    pros::delay(300);
    chassis->moveToPoint(-54.423, 24.824, 2000, {.forwards = false, .minSpeed = 127});
    chassis->waitUntil(1);
    chassis->cancelMotion();
    chassis->moveToPoint(-36.835, 22.867, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 5});
    arm.set_state(mechanism::ArmState::PRIME);

    // mogo
    chassis->moveToPoint(-22.504, 25.0, 2000, {.forwards = false, .maxSpeed = 90});
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
    chassis->waitUntil(20);
    chassis->cancelMotion();

    // middle ring
    chassis->swingToPoint(-23.5, 48, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 60, .earlyExitRange = 30});
    chassis->moveToPoint(-23.5, 48, 2000, {.minSpeed = 20, .earlyExitRange = 7});

    chassis->turnToPoint(-60, 60, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 40, .earlyExitRange = 20z});
    doinker_right.extend();
    arm.set_state(mechanism::ArmState::ALLIANCE_STAKE);

    // corner
    chassis->moveToPoint(-58.79, 58.278, 2000, {.minSpeed = 127, .earlyExitRange = 12});
    doinker_right.retract();
    chassis->moveToPoint(-72.79, 72.278, 800, {.minSpeed = 127});

    chassis->moveToPoint(-50, 50, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 3});
    arm.set_state(mechanism::ArmState::PRIME);
    clamp.set_autoclamp(false);
    
    // alliance ring
    chassis->moveToPoint(-48, 15, 2000, {.minSpeed = 127, .earlyExitRange = 14});
    doinker_left.extend();
    chassis->moveToPose(-48, 0, -180, 2000, {.lead = 0.1, .maxSpeed = 60});
    intake.enable_stop_next_ring();
    doinker_left.retract();
    intake_lift.extend();
    pros::delay(300);
    clamp.retract();
    chassis->waitUntil(21);
    intake_lift.retract();
    chassis->cancelMotion();

    // clamp
    chassis->turnToPoint(-24, -21.5, 2000, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 60, .earlyExitRange = 30});
    chassis->moveToPoint(-24, -21.5, 2000, {.forwards = false});
    pros::delay(300);
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        
        pros::delay(10);
    }
    chassis->cancelMotion();
    clamp.extend();
    intake.disable_stop_next_ring();
    
    // corner rings
    chassis->moveToPoint(-54, -30, 2000, {.minSpeed = 40, .earlyExitRange = 13});
    pros::delay(500);
    intake.set_state(mechanism::IntakeState::HOOK);
    arm.set_state(mechanism::ArmState::ALLIANCE_STAKE);
    chassis->moveToPoint(-66.79, -60.278, 2000, {.minSpeed = 127, .earlyExitRange = 12});
    chassis->moveToPoint(-72.79, -80.278, 800, {.minSpeed = 127});

    chassis->moveToPoint(-47, -40, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 8});
    doinker_left.extend();

    chassis->moveToPoint(-24, -48, 2000, {.minSpeed = 127, .earlyExitRange = 5});
    arm.set_state(mechanism::ArmState::PRIME);
    pros::delay(500);
    doinker_left.retract();
    chassis->moveToPoint(-36, -40.5, 2000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 8});
    chassis->moveToPose(-20, -16, 45, 2000, {.minSpeed = 60});
    arm.set_state(mechanism::ArmState::TOUCH);
    if (pros::millis() > 1700) {
        arm.set_state(mechanism::ArmState::ALLIANCE_STAKE);
    }
}