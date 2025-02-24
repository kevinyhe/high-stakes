#include "autonomous.hpp"
#include "config.hpp"
#include "main.h"

void red_mogo()
{
    chassis->setPose(-51.5, -58.5, 90);
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);


    // intake ring on path
    intake.set_state(mechanism::IntakeState::FIRST_HOOK);
    chassis->moveToPoint(-17.344, -45.571, 3000);
    chassis->waitUntilDone();
    doinker.extend();

    chassis->moveToPoint(-40, -48, 3000, {.forwards = false});
}