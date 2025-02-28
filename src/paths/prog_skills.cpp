#include "config.hpp"
#include "autonomous.hpp"
#include "main.h"

void tap() {
    auto &intake = mechanism::Intake::get_instance();

    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(250);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(150);
    intake.set_state(mechanism::IntakeState::HOOK);
    pros::delay(250);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(150);
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
    chassis->moveToPoint(-48, -28, 3000, {.forwards = false, .maxSpeed = 65});
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
    chassis->moveToPoint(29, -48, 2000, {.minSpeed = 5, .earlyExitRange = 5});
    // chassis->moveToPoint(46, -58, 2000);
    chassis->waitUntil(16);
    // chassis->moveToPoint(44, -58, 3000);
    // chassis->waitUntil(20);
    arm.set_state(mechanism::ArmState::LOAD);
    
    // wall stake
    chassis->moveToPoint(0, -38, 3000, {.forwards=false, .minSpeed = 20, .earlyExitRange = 5});
    // chassis->moveToPoint(0, -68, 7000, {.maxSpeed = 90});
    chassis->moveToPose(0, -62.5, 180, 2000, {.lead = 0.3});
    tap();
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    // pros::delay(600);
    chassis->waitUntilDone();
    chassis->moveToPoint(0, -72, 5000, {.maxSpeed = 45});
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(600);
    chassis->cancelMotion();
    chassis->setPose(chassis->getPose().x, -60.75, chassis->getPose().theta);
    pros::delay(100);
    chassis->moveToPoint(-0, -44, 1500, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2});
    pros::delay(500);
    arm.set_state(mechanism::ArmState::PRIME);
    // distance = wall_reset.get() * 0.0393701; // to inches
    
    // second set of rings
    intake.set_state(mechanism::IntakeState::HOOK);
    // chassis->swingToPoint(-64, -48, lemlib::DriveSide::LEFT, 3000, {.minSpeed = 10, .earlyExitRange = 20});
    chassis->moveToPoint(-40, -48, 2000, {.minSpeed = 90, .earlyExitRange = 12});
    chassis->moveToPoint(-65, -48, 1300, {.maxSpeed = 60});
    chassis->waitUntilDone();
    chassis->setPose(-62.25, chassis->getPose().y, chassis->getPose().theta);
    pros::delay(200);
    chassis->moveToPoint(-47, -39, 3000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 4});
    chassis->moveToPoint(-51, -60, 3000, {.minSpeed = 40, .earlyExitRange = 4});

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
    chassis->moveToPoint(-48, -10, 2000, {.minSpeed = 72, .earlyExitRange = 4});
    intake.set_state(mechanism::IntakeState::DISABLED);
    arm.set_state(mechanism::ArmState::IDLE);
    chassis->moveToPoint(-48, 28, 5000, {.forwards = false, .maxSpeed = 55});
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
    chassis->moveToPoint(-6, 42, 2000, {.maxSpeed = 85, .minSpeed = 20, .earlyExitRange = 15}); // evade barrier
    chassis->moveToPoint(29, 48, 2000, {.minSpeed = 5, .earlyExitRange = 5});
    // chassis->moveToPoint(46, -58, 2000);
    chassis->waitUntil(16);
    // chassis->moveToPoint(44, -58, 3000);
    // chassis->waitUntil(20);
    arm.set_state(mechanism::ArmState::LOAD);

    // wall stake
    chassis->moveToPoint(0, 38, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 5});
    // chassis->moveToPoint(0, -68, 7000, {.maxSpeed = 90});
    chassis->moveToPose(0, 62.5, 0, 2000, {.lead = 0.3});
    tap();
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    // pros::delay(600);
    chassis->waitUntilDone();
    chassis->moveToPoint(0, 72, 5000, {.maxSpeed = 45});
    arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
    intake.set_state(mechanism::IntakeState::WALL_STAKE);
    pros::delay(600);
    chassis->cancelMotion();
    chassis->setPose(chassis->getPose().x, 60.75, chassis->getPose().theta);
    pros::delay(100);
    chassis->moveToPoint(-0, 44, 1500, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2});
    pros::delay(500);
    arm.set_state(mechanism::ArmState::PRIME);

    // second set of rings
    intake.set_state(mechanism::IntakeState::HOOK);
    // chassis->swingToPoint(-64, 48, lemlib::DriveSide::RIGHT, 3000, {.minSpeed = 10, .earlyExitRange = 20});
    chassis->moveToPoint(-40, 48, 2000, {.minSpeed = 90, .earlyExitRange = 12});
    // chassis->moveToPoint(-36, 48, 3000, {.minSpeed = 20, .earlyExitRange = 12});
    chassis->moveToPoint(-65, 48, 1500, {.maxSpeed = 60});
    chassis->waitUntilDone();
    chassis->setPose(-62.25, chassis->getPose().y, chassis->getPose().theta);
    pros::delay(200);
    chassis->moveToPoint(-47, 39, 3000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 4});
    chassis->moveToPoint(-51, 60, 3000, {.minSpeed = 40, .earlyExitRange = 4});

    // drop mogo
    chassis->moveToPoint(-62, 62, 1200, {.forwards = false});
    pros::delay(800);
    intake.set_state(mechanism::IntakeState::REVERSE);
    pros::delay(200);
    clamp.set_autoclamp(false);
    clamp.retract();
    chassis->waitUntilDone();
    
    pros::delay(200);
    intake.enable_sort(mechanism::Intake::RingColours::BLUE);
    
    /**
     * THIRD QUARTER
    */
    // third mogo
    chassis->moveToPoint(43, 48, 4000, {.minSpeed = 72, .earlyExitRange = 4});
    arm.set_state(mechanism::ArmState::IDLE);
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    chassis->moveToPoint(39, 29, 4000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 10});

    chassis->moveToPoint(48, 1, 4000, {.forwards = false, .maxSpeed = 70});
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
    chassis->moveToPoint(26, -20.5, 3000, {.minSpeed = 5, .earlyExitRange = 6});
    // under ladder
    chassis->moveToPoint(3, -3, 3000, {.maxSpeed = 80,  .minSpeed = 10, .earlyExitRange = 4});
    pros::delay(700);
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    chassis->moveToPoint(24, 24, 3000, {.minSpeed = 72, .earlyExitRange = 4});
    chassis->waitUntil(30);
    intake.set_state(mechanism::IntakeState::HOOK);

    // last 2
    chassis->moveToPoint(56, 46, 3000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis->moveToPoint(48, 48, 2000, {.minSpeed = 20, .earlyExitRange = 2});
    chassis->moveToPoint(46, 65, 1900, {.minSpeed = 20, .earlyExitRange = 3});
    // chassis->waitUntilDone();
    // chassis->setPose(chassis->getPose().x, 61.75, chassis->getPose().theta);

    // corner
    chassis->moveToPoint(63.5, 40, 3000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 5});
    doinker.extend();
    chassis->moveToPoint(63.5, 62, 1500, {.minSpeed = 20, .earlyExitRange = 5});
    // chassis->moveToPose(62.5, 62, 5, 1500);
    chassis->turnToHeading(-40, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed=72, .earlyExitRange = 10});
    // intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    chassis->moveToPoint(72, 72, 1500, {.forwards = false});
    doinker.retract();
    pros::delay(800);
    intake.set_state(mechanism::IntakeState::REVERSE);
    pros::delay(200);
    clamp.set_autoclamp(false);
    clamp.retract();
    chassis->waitUntilDone();

    // blue mogo
    chassis->moveToPoint(37,26, 3000, {.minSpeed= 70, .earlyExitRange = 4});
    // chassis->swingToPoint(59, -20.5, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 72, .earlyExitRange = 20});
    chassis->moveToPoint(59, -20.5, 3000, {.forwards = false});
    clamp.set_autoclamp(true);
    while (chassis->isInMotion() && !clamp.get_value())
    {
        pros::delay(20);
    }
    chassis->cancelMotion();
    clamp.extend();
    pros::delay(200);

    // intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    chassis->moveToPoint(62, -62, 2000, {.forwards = false});
    chassis->waitUntilDone();
    // intake.set_state(mechanism::IntakeState::REVERSE);
    clamp.set_autoclamp(false);
    clamp.retract();
    
    // alliance stakes
    chassis->moveToPoint(56, -24, 3000, {.minSpeed = 20, .earlyExitRange = 5});
    // chassis->moveToPoint(48, -48, 3000);
    chassis->moveToPoint(64, -48, 2300, {.minSpeed = 20, .earlyExitRange = 2});
    // chassis->moveToPoint(47, -40, 3000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 4});
    // chassis->turnToHeading(80, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.minSpeed = 72, .earlyExitRange = 10});
    // intake.set_state(mechanism::IntakeState::WALL_STAKE);
    // doinker.extend();

    // intake.set_state(mechanism::IntakeState::DISABLED);

    chassis->moveToPoint(58, 0, 3000);
    chassis->turnToPoint(65, 0, 2000, {.forwards = false});
    chassis->moveToPoint(65, 0, 3000, {.forwards = false});
    chassis->waitUntilDone();
    intake.set_state(mechanism::IntakeState::HOOK);

    pros::delay(1000);
    intake.set_state(mechanism::IntakeState::DISABLED);
}
