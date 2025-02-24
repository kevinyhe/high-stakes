#include "config.hpp"
#include "autonomous.hpp"
#include "main.h"

void prog_skills()
{
    chassis->setPose(-64.625, 0, 90);

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
    chassis->moveToPoint(-48, -24, 3000, {.forwards = false, .maxSpeed = 70});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {

        pros::delay(20);
    }
    chassis->cancelMotion();
    pros::delay(200);

    intake.set_state(mechanism::IntakeState::HOOK);

    // first set of rings
    chassis->moveToPoint(-28, -24, 2000, {.maxSpeed = 100, .minSpeed = 72, .earlyExitRange = 8});
    chassis->moveToPoint(5, -49, 2000, {.minSpeed = 20, .earlyExitRange = 20}); // evade barrier
    chassis->moveToPoint(28, -48, 2000, {.minSpeed = 20, .earlyExitRange = 5});
    // chassis->moveToPoint(46, -58, 2000);
    chassis->waitUntilDone();
    // chassis->moveToPoint(44, -58, 3000);
    // chassis->waitUntil(20);
    arm.set_state(mechanism::ArmState::LOAD);

    // wall stake
    chassis->moveToPoint(-3, -42, 3000, {.forwards=false});
    chassis->moveToPoint(-3, -65, 3000);
    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(200);
    intake.set_state(mechanism::IntakeState::DISABLED);
    pros::delay(100);
    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(200);
    intake.set_state(mechanism::IntakeState::DISABLED);
    pros::delay(100);
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->waitUntilDone();
    chassis->tank(20, 20);
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(1200);
    // chassis->tank(0, 0);
    arm.set_state(mechanism::ArmState::IDLE);
    // double distance = wall_reset.get() * 0.0393701; // to inches

    // lemlib::Pose pose = chassis->getPose();
    // chassis->setPose(pose.x, -62+distance, pose.theta);

    pros::delay(200);

    // second set of rings
    chassis->swingToPoint(-11, -52.5, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 20, .earlyExitRange = 5});
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPoint(-17, -48, 3000, {.minSpeed = 72, .earlyExitRange = 6});
    chassis->moveToPoint(-56, -48, 3000, {.maxSpeed = 70});
    chassis->moveToPoint(-47, -41, 3000, {.forwards = false, .minSpeed = 30});
    chassis->moveToPoint(-48, -60, 3000);

    // drop mogo
    chassis->moveToPoint(-72, -72, 1500, {.forwards = false});
    chassis->waitUntilDone();
    intake.set_state(mechanism::IntakeState::REVERSE);
    pros::delay(200);

    clamp.set_autoclamp(false);
    clamp.retract();
    
    // second mogo
    chassis->moveToPoint(-48, -10, 5000);
    intake.set_state(mechanism::IntakeState::DISABLED);
    chassis->moveToPoint(-48, 24, 5000, {.forwards = false, .maxSpeed = 70});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {

        pros::delay(20);
    }
    chassis->cancelMotion();
    pros::delay(200);

    intake.set_state(mechanism::IntakeState::HOOK);
    
    // third set of rings
    chassis->moveToPoint(-28, 24, 2000, {.maxSpeed = 80, .minSpeed = 72, .earlyExitRange = 6});
    chassis->moveToPoint(5, 47, 2000, {.minSpeed = 72, .earlyExitRange = 20}); // evade barrier
    chassis->moveToPoint(26, 48, 2000, {.minSpeed = 20, .earlyExitRange = 5});
    // chassis->moveToPoint(44, -58, 3000);
    chassis->waitUntil(20);
    arm.set_state(mechanism::ArmState::LOAD);

    // wall stake
    chassis->moveToPoint(-3.4, 42, 3000, {.forwards=false});
    chassis->moveToPoint(-3.4, 65, 3000);
    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(200);
    intake.set_state(mechanism::IntakeState::DISABLED);
    pros::delay(100);
    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(200);
    intake.set_state(mechanism::IntakeState::DISABLED);
    pros::delay(100);
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->waitUntilDone();
    chassis->tank(20, 20);
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(1000);
    // chassis->tank(0, 0);
    arm.set_state(mechanism::ArmState::IDLE);
    pros::delay(200);

    // second set of rings
    chassis->swingToPoint(-11, 52.5, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 20, .earlyExitRange = 5});
    intake.set_state(mechanism::IntakeState::HOOK);
    chassis->moveToPoint(-17, 48, 3000, {.minSpeed = 72, .earlyExitRange = 6});
    chassis->moveToPoint(-56, 48, 3000, {.maxSpeed = 90});
    chassis->moveToPoint(-47, 41, 3000, {.forwards = false, .minSpeed = 72});
    chassis->moveToPoint(-48, 60, 3000);

    // drop mogo
    chassis->moveToPoint(-70, 70, 3000, {.forwards = false});
    chassis->waitUntilDone();
    
    intake.set_state(mechanism::IntakeState::REVERSE);
    clamp.set_autoclamp(false);
    clamp.retract();
}
