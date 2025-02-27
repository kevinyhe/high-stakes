#include "config.hpp"
#include "autonomous.hpp"
#include "main.h"

void tap() {
    auto &intake = mechanism::Intake::get_instance();

    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(200);
    intake.set_state(mechanism::IntakeState::DEJAM);
    pros::delay(200);
    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(200);
    intake.set_state(mechanism::IntakeState::DEJAM);
    pros::delay(200);
    intake.set_state(mechanism::IntakeState::HOOK);
}

void prog_skills()
{
    double distance;
    chassis->setPose(-62.825, 0, 90);

    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(500);
    intake.set_state(mechanism::IntakeState::DISABLED);

    // clamp first mogo
    chassis->moveToPoint(-48, 0, 1000, {.minSpeed = 20, .earlyExitRange = 8});
    // chassis->swingToPoint(-48, -24, lemlib::DriveSide::LEFT, 3000, {.forwards = false});
    chassis->moveToPoint(-48, -28, 3000, {.forwards = false, .maxSpeed = 75});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {

        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();

    pros::delay(200);

    intake.set_state(mechanism::IntakeState::HOOK);

    // first set of rings
    chassis->moveToPoint(-28, -24, 2000, {.maxSpeed = 100, .minSpeed = 72, .earlyExitRange = 8});
    chassis->moveToPoint(-6, -38, 2000, {.maxSpeed = 85, .minSpeed = 20, .earlyExitRange = 15}); // evade barrier
    chassis->moveToPoint(32, -48, 2000, {.minSpeed = 20, .earlyExitRange = 5});
    // chassis->moveToPoint(46, -58, 2000);
    chassis->waitUntil(22);
    // chassis->moveToPoint(44, -58, 3000);
    // chassis->waitUntil(20);
    arm.set_state(mechanism::ArmState::LOAD);

    // wall stake
    chassis->moveToPoint(0, -40, 3000, {.forwards=false, .minSpeed = 20, .earlyExitRange = 5});
    // chassis->moveToPoint(0, -68, 7000, {.maxSpeed = 90});
    chassis->moveToPose(0, -68, 180, 2000);
    tap();
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    // pros::delay(600);
    chassis->waitUntilDone();
    chassis->moveToPoint(0, -72, 5000, {.maxSpeed = 20});
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(600);
    chassis->cancelMotion();
    distance = wall_reset.get() * 0.0393701; // to inches
    chassis->setPose(0, -59.3, chassis->getPose().theta);
    chassis->moveToPoint(-0, -50, 1500, {.forwards = false});
    pros::delay(500);
    arm.set_state(mechanism::ArmState::PRIME);
    // distance = wall_reset.get() * 0.0393701; // to inches

    // second set of rings
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->swingToPoint(-48, -47, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 80, .earlyExitRange = 30});
    chassis->moveToPoint(-65, -48, 2500, {.maxSpeed = 60});
    chassis->waitUntilDone();
    chassis->setPose(-61.75, chassis->getPose().y, chassis->getPose().theta);
    chassis->moveToPoint(-47, -39, 3000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 4});
    chassis->moveToPoint(-51, -60, 3000);

    pros::delay(300);

    // drop mogo
    chassis->moveToPoint(-62, -62, 1200, {.forwards = false});
    pros::delay(800);
    intake.set_state(mechanism::IntakeState::REVERSE);
    pros::delay(200);
    clamp.set_autoclamp(false);
    clamp.retract();
    chassis->waitUntilDone();
    
    pros::delay(200);
    
    // second mogo
    chassis->moveToPoint(-48, -10, 2000);
    intake.set_state(mechanism::IntakeState::DISABLED);
    arm.set_state(mechanism::ArmState::IDLE);
    chassis->moveToPoint(-48, 28, 5000, {.forwards = false, .maxSpeed = 70});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();
    pros::delay(200);

    intake.set_state(mechanism::IntakeState::HOOK);

    /**
     * SECOND QUARTER
     */
    // first set of rings
    chassis->moveToPoint(-28, 24, 2000, {.maxSpeed = 100, .minSpeed = 72, .earlyExitRange = 8});
    chassis->moveToPoint(-6, 38, 2000, {.maxSpeed = 85, .minSpeed = 20, .earlyExitRange = 15}); // evade barrier
    chassis->moveToPoint(32, 48, 2000, {.minSpeed = 20, .earlyExitRange = 5});
    // chassis->moveToPoint(46, -58, 2000);
    chassis->waitUntil(16);
    // chassis->moveToPoint(44, -58, 3000);
    // chassis->waitUntil(20);
    arm.set_state(mechanism::ArmState::LOAD);

    // wall stake
    chassis->moveToPoint(0, 40, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 5});
    // chassis->moveToPoint(0, -68, 7000, {.maxSpeed = 90});
    chassis->moveToPose(0, 68, 0, 2000);
    tap();
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    // pros::delay(600);
    chassis->waitUntilDone();
    chassis->moveToPoint(0, 72, 5000, {.maxSpeed = 20});
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(600);
    chassis->cancelMotion();
    chassis->setPose(0, 59.3, chassis->getPose().theta);
    chassis->moveToPoint(0, 50, 1500, {.forwards = false});
    pros::delay(500);
    arm.set_state(mechanism::ArmState::PRIME);

    pros::delay(200);
    
    // second set of rings
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->swingToPoint(-48, 48, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 80, .earlyExitRange = 30});
    chassis->moveToPoint(-65, 48, 2500, {.maxSpeed = 60});
    chassis->waitUntilDone();
    chassis->setPose(-61.75, chassis->getPose().y, chassis->getPose().theta);
    chassis->moveToPoint(-47, 39, 3000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 4});
    chassis->moveToPoint(-51, 60, 3000);

    pros::delay(300);

    // drop mogo
    chassis->moveToPoint(-62, 62, 1200, {.forwards = false});
    pros::delay(800);
    intake.set_state(mechanism::IntakeState::REVERSE);
    pros::delay(200);
    clamp.set_autoclamp(false);
    clamp.retract();
    chassis->waitUntilDone();

    pros::delay(200);

    // third mogo
    chassis->moveToPoint(48, 48, 5000);
    chassis->waitUntil(48);
    intake.set_state(mechanism::IntakeState::HOOK);
    intake.stop_next_ring();
}
