#include "config.hpp"
#include "autonomous.hpp"
#include "main.h"

void tap() {
    auto &intake = mechanism::Intake::get_instance();

    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(150);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(150);
    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(150);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(150);
    intake.set_state(mechanism::IntakeState::HOOK);
}

void prog_skills()
{
    chassis->setPose(-62.825, 0, 90);

    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    pros::Distance wall_reset(15);

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
    chassis->moveToPoint(0, -44, 3000, {.forwards=false});
    // chassis->moveToPoint(0, -68, 7000, {.maxSpeed = 90});
    chassis->moveToPose(0, -68, 180, 1500);
    chassis->waitUntil(4);
    tap();
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    // pros::delay(400);
    chassis->waitUntilDone();
    chassis->moveToPoint(0, -72, 5000, {.maxSpeed = 20});
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(300);
    chassis->cancelMotion();
    chassis->moveToPoint(-0, -54, 1500);
    pros::delay(300);
    arm.set_state(mechanism::ArmState::PRIME);
    pros::delay(200);
    // double distance = wall_reset.get() * 0.0393701; // to inches

    lemlib::Pose pose = chassis->getPose();
    chassis->setPose(0, pose.y, pose.theta);
    pros::delay(200);

    // second set of rings
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->swingToPoint(-48, -48, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 80, .earlyExitRange = 40});
    chassis->moveToPoint(-48, -48, 3000, {.minSpeed = 10, .earlyExitRange = 12});
    chassis->moveToPoint(-60, -48, 3000, {.maxSpeed = 60});
    chassis->moveToPoint(-47, -39, 3000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 2});
    chassis->moveToPoint(-49, -60, 3000);

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
    chassis->moveToPoint(-48, -10, 5000);
    intake.set_state(mechanism::IntakeState::DISABLED);
    arm.set_state(mechanism::ArmState::IDLE);
    chassis->moveToPoint(-48, 24, 5000, {.forwards = false, .maxSpeed = 70});
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
    chassis->waitUntil(22);
    // chassis->moveToPoint(44, -58, 3000);
    // chassis->waitUntil(20);
    arm.set_state(mechanism::ArmState::LOAD);

    // wall stake
    chassis->moveToPoint(0, 44, 3000, {.forwards = false});
    // chassis->moveToPoint(0, -68, 7000, {.maxSpeed = 90});
    chassis->moveToPose(0, 68, 0, 1500);
    chassis->waitUntil(4);
    tap();
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    // pros::delay(400);
    chassis->waitUntilDone();
    chassis->moveToPoint(0, 72, 5000, {.maxSpeed = 20});
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(300);
    pose = chassis->getPose();
    chassis->setPose(0, pose.y, pose.theta);
    chassis->cancelMotion();
    chassis->moveToPoint(-0, 54, 1500);
    pros::delay(300);
    arm.set_state(mechanism::ArmState::PRIME);
    pros::delay(200);
    // double distance = wall_reset.get() * 0.0393701; // to inches

    // lemlib::Pose pose = chassis->getPose();
    // chassis->setPose(pose.x, -62+distance, pose.theta);
    pros::delay(200);

    // second set of rings
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->swingToPoint(-48, 48, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 80, .earlyExitRange = 40});
    chassis->moveToPoint(-48, 48, 3000, {.minSpeed = 10, .earlyExitRange = 12});
    chassis->moveToPoint(-60, 48, 3000, {.maxSpeed = 60});
    chassis->moveToPoint(-47, 39, 3000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 2});
    chassis->moveToPoint(-49, 60, 3000);

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
}
